//
// HostInterface.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Threading;
using System.Collections.Concurrent;
using System.Net.NetworkInformation;

using SharpPcap;
using PacketDotNet;
using PacketDotNet.Utils;

using PERQemu.Config;

namespace PERQemu.IO.Network
{
    /// <summary>
    /// Encapsulate a host Ethernet interface for sending and receiving PERQ
    /// packets on a real network.
    /// </summary>
    public class HostAdapter
    {
        public HostAdapter(INetworkController controller, string devName)
        {
            _nat = new NATTable();
            _pending = new ConcurrentQueue<EthernetPacket>();

            _controller = controller;
            _adapter = GetAdapter(devName);

            if (_adapter == null)
            {
                throw new UnimplementedHardwareException("Host adapter not found (or not accessible)");
            }

            // Open the device and attach our receive handler
            _adapter.Open(DeviceMode.Promiscuous);
            _adapter.OnPacketArrival += OnPacketArrival;

            // This _seems_ to avoid the spurious exception on shutdown
            _adapter.StopCaptureTimeout = new TimeSpan(100000000);

            // Set up a timer for periodic aging/refresh of the NAT table
            _natRefreshTimer = HighResolutionTimer.Register(60000d, DoNATRefresh, "NAT");

            // Initialize statistics
            _probed = _hasFCS = false;
            _pktsSent = _pktsRecvd = _pktsIgnored = _pktsQueued = _pktsDropped = 0;

            Log.Info(Category.NetAdapter, "Device opened [Host MAC: {0}]", _adapter.MacAddress);
        }

        public string Name => _adapter?.Name;
        public string Description => _adapter?.Description;

        public PhysicalAddress Address => (_adapter == null ? PhysicalAddress.None : _adapter.MacAddress);

        public bool Running => (_adapter != null && _adapter.Started);
        public bool FrameIncludesFCS => _hasFCS;


        /// <summary>
        /// There's nothing to reset, really; we just use this to lazily start
        /// packet capture once the rest of the virtual PERQ is set up.  Any
        /// subsequent calls will send out a "greeting", periodically.
        /// </summary>
        public void Reset()
        {
            if (!Running)
            {
                // If we just changed configuration, our own address may have changed;
                // the table will get refreshed pretty quickly so this shouldn't be too
                // disruptive (but make it a CLI option instead, if it is)
                _nat.Flush();

                // Add our local NAT entry
                if (!_nat.Add(new NATEntry(_adapter.MacAddress, _controller.MACAddress, Flags.Me)))
                {
                    Log.Warn(Category.All, "Another PERQ detected with our MAC address!?");
                }

                // Enable our refresh timer (looooow priority)
                HighResolutionTimer.Enable(_natRefreshTimer, true);

                // Fire up the receive thread
                _adapter.StartCapture();

                Log.Info(Category.NetAdapter, "Adapter reset (packet capture started)");
                return;
            }

            // Announce our presence with authori-tie
            SendGreeting();
        }

        /// <summary>
        /// Flush oldest received packets to keep queue from growing too long.
        /// </summary>
        public void Flush()
        {
            int max = _pending.Count;
            int count = 0;

            // Todo: check a timestamp, or maybe keep most recent packet?  Or
            // ditch broadcasts before any sent to us directly?

            // POS programs must explicitly be in a polling mode and may ignore
            // incoming traffic forever; Accent is more modern in that it tries
            // to dispatch packets as they arrive so queues shouldn't back up;
            // not sure about PNX (not enough testing/experience there).
            while (_pending.Count >= MaxBacklog)
            {
                EthernetPacket tossIt;
                if (_pending.TryDequeue(out tossIt))
                {
                    count++;
                    _pktsDropped++;
                }
            }

            if (count > 0)
                Log.Info(Category.NetAdapter, "Max backlog exceeded ({0}), flushed {1} packet(s)", max, count);
        }

        /// <summary>
        /// Send a RARP request with our emulated address to let other PERQemu
        /// (or real PERQ) nodes know we're here.  We don't expect a reply, but
        /// issue these periodically just to check in.  Someday this might be
        /// formalized into a way to allow PERQs to rendezvous over the Interwebs!
        /// </summary>
        public void SendGreeting()
        {
            // Don't spam the network with broadcasts!
            TimeSpan ts = DateTime.Now - _lastGreeting;
            if (ts.TotalSeconds < GreetingInterval) return;

            // Todo: would be nice to be able to flag our first greeting in case
            // our address changed so others know to invalidate their old mapping
            try
            {
                // Broadcast our request
                var packet = new EthernetPacket(_adapter.MacAddress,
                                                Broadcast,
                                                EthernetType.ReverseArp);
                // Well hello there!
                var greeting = new ArpPacket(ArpOperation.RequestReverse,
                                             _controller.MACAddress,
                                             System.Net.IPAddress.None,
                                             _adapter.MacAddress,
                                             System.Net.IPAddress.None);

                packet.PayloadPacket = greeting;
                _adapter.SendPacket(packet);
                _pktsSent++;
                _lastGreeting = DateTime.Now;

                Log.Info(Category.Network, "Sent RARP request from {0}", _controller.MACAddress);
            }
            catch (PcapException ex)
            {
                Log.Error(Category.Network, "Failed to send greeting packet: {0}", ex.Message);
            }
        }

        /// <summary>
        /// Send a RARP reply when we catch one from another PERQ.  We're kinda
        /// breaking the paradigm a little bit; PERQemu uses the "request" to
        /// tell other PERQs out there about us, while the "reply" here is to
        /// say "gotcha, here's my info in return".  That way when a new instance
        /// comes online it quickly gathers up data for the others without doing
        /// periodic broadcasts.  This is all kinda cheesy. :-)
        /// </summary>
        void SendReply(ArpPacket greeting)
        {
            try
            {
                // Return to sender
                var packet = new EthernetPacket(_adapter.MacAddress,
                                                greeting.SenderHardwareAddress,
                                                EthernetType.ReverseArp);
                // Send back our data
                var salutation = new ArpPacket(ArpOperation.ReplyReverse,
                                              _controller.MACAddress,
                                               System.Net.IPAddress.None,
                                              _adapter.MacAddress,
                                               System.Net.IPAddress.None);

                packet.PayloadPacket = salutation;
                _adapter.SendPacket(packet);
                _pktsSent++;

                Log.Info(Category.Network, "Sent RARP reply to {0}", greeting.TargetHardwareAddress);
            }
            catch (PcapException ex)
            {
                Log.Error(Category.Network, "Failed to send greeting reply: {0}", ex.Message);
            }
        }

        /// <summary>
        /// Send a raw Ethernet packet straight from the PERQ, baybee!!
        /// </summary>
        public bool SendPacket(byte[] raw)
        {
            try
            {
                // Turn raw bytes from the PERQ provided into a packet
                var packet = new EthernetPacket(new ByteArraySegment(raw));
                if (packet == null)
                {
                    Log.Error(Category.NetAdapter, "Could not format packet on send");
                    // Dump more data (if this ever happens); drop it like it's hot
                    return false;
                }

                Log.Info(Category.NetAdapter, "Sending from {0} to {1} (type 0x{2:x})",
                          packet.SourceHardwareAddress, packet.DestinationHardwareAddress, packet.Type);
                Log.Info(Category.NetAdapter, "SIZES: packet {0}, header {1}, payload {2}",
                          packet.Bytes.Length, packet.HeaderData.Length, packet.PayloadData?.Length);

                // Always remap our source address to the host adapter
                packet.SourceHardwareAddress = _adapter.MacAddress;

                // Are we (potentially) sending to another PERQ?
                if (IsPerqPrefix(packet.DestinationHardwareAddress) ||
                    packet.DestinationHardwareAddress.Equals(Broadcast))
                {
                    if (!packet.DestinationHardwareAddress.Equals(Broadcast))
                    {
                        // Look up the PERQ's host address
                        var map = _nat.LookupPerq(packet.DestinationHardwareAddress);

                        if (map != null)
                        {
                            // Translate it too
                            packet.DestinationHardwareAddress = map.Host;

                            // Basic stats
                            map.Sent++;

                            Log.Info(Category.Network, "Send to Perq {0} via Host {1}", map.Perq, map.Host);
                        }
                        else
                        {
                            Log.Warn(Category.Network, "Destination Perq {0} is unknown to me...", packet.DestinationHardwareAddress);
                            // Should do an actual RARP here...?
                        }
                    }

                    // Retranslate the EtherType/Length field if necessary
                    var perqType = PortMap((ushort)packet.Type);

                    if ((ushort)packet.Type != perqType)
                    {
                        Log.Info(Category.Network, "EtherType mapped from 0x{0:x4} to 0x{1:x4}", (ushort)packet.Type, perqType);
                        packet.Type = (EthernetType)perqType;
                    }
                }

#if DEBUG
                // Debugging: generate the checksum for the packet, post-rewrites
                var crc = Crc32.Compute(packet.Bytes, 0, packet.Bytes.Length);
                Log.Debug(Category.NetAdapter, "Computed CRC is {0:x8}", crc);

                // In verbose mode print the (modified) packet
                if (Log.Level < Severity.Detail) Console.WriteLine(packet.PrintHex());
#endif

                // So send it already, sheesh
                _adapter.SendPacket(packet);
                _pktsSent++;
                return true;
            }
            catch (PcapException ex)
            {
                Log.Error(Category.NetAdapter, "Failed to send packet: {0}", ex.Message);
                return false;
            }
        }

        /// <summary>
        /// Callback for incoming packets:  make sure a valid Ethernet frame is
        /// received, perform NAT or handle RARP processing if appropriate, then
        /// ask the PERQ controller if it wants to handle the packet.  If yes,
        /// queue it up; if no, or if the queue is full, drop the packet.  The
        /// PERQ only expects a 10Mbit/half-duplex level of traffic, so there's
        /// no realistic expectation of "wire speed" levels of throughput here.
        /// </summary>
        void OnPacketArrival(object s, CaptureEventArgs e)
        {
            if (e.Packet.LinkLayerType != LinkLayers.Ethernet)
            {
                _pktsIgnored++;
                Log.Warn(Category.NetAdapter, "Non-Ethernet packet type {0} ignored", e.Packet.LinkLayerType);
                return;
            }

            //
            // Start with the raw Ethernet frame
            //
            EthernetPacket raw;

            try
            {
                raw = (EthernetPacket)Packet.ParsePacket(e.Packet.LinkLayerType, e.Packet.Data);
                if (raw == null)
                {
                    Log.Warn(Category.NetAdapter, "Failed to parse packet: {0}", e.Packet);
                    return;
                }

                // The PERQ interface can't "see" its own transmissions, but
                // apparently SharpPcap does; silently drop 'em here
                if (raw.SourceHardwareAddress.Equals(_adapter.MacAddress)) return;

                // Stuff we just drop because it's completely irrelevant to the
                // old PERQ and is just pure noise:  IPv6 and spanning tree
                // multicasts every 2 seconds... there's MUCH more we could add
                // but it might be simpler to just set a filter for what we can
                // safely accept?
                if (raw.Type == EthernetType.IPv6) return;
                if ((ushort)raw.Type == 0x0026) return;

                // Log it
                Log.Debug(Category.NetAdapter, "Received from {0} to {1} (type 0x{2:x}) [{3}]",
                          raw.SourceHardwareAddress, raw.DestinationHardwareAddress, raw.Type,
                          Thread.CurrentThread.ManagedThreadId);
                Log.Detail(Category.NetAdapter, "SIZES: packet {0}, header {1}, payload {2}",
                          raw.Bytes.Length, raw.HeaderData.Length, raw.PayloadData?.Length);

                // Check if this interface needs some special CRC handling
                ProbeFCS(raw);

#if DEBUG
                // Debugging: Print the packet pre-rewrites (extremely verbose)
                if (Log.Level < Severity.Detail) Console.WriteLine(raw.PrintHex());
#endif

                // If this is addressed to us specifically, NAT it!
                if (raw.DestinationHardwareAddress.Equals(_adapter.MacAddress))
                {
                    raw.DestinationHardwareAddress = _controller.MACAddress;
                }

                // Is it from a PERQ that we've seen before?
                var src = _nat.LookupHost(raw.SourceHardwareAddress);

                if (src != null)
                {
                    // Yes!  Translate the source address too
                    raw.SourceHardwareAddress = src.Perq;

                    // Update the entry to show they're still active
                    src.UpdateReceived();

                    Log.Debug(Category.Network, "NAT receive from Perq {0} via Host {1}", src.Perq, src.Host);
                }

                // If source is a PERQ, see if the Type/Length field needs remappin'
                if (IsPerqPrefix(raw.SourceHardwareAddress) || raw.DestinationHardwareAddress.Equals(Broadcast))
                {
                    // Translate the EtherType/Length field if necessary
                    var perqType = PortMap((ushort)raw.Type);

                    if ((ushort)raw.Type != perqType)
                    {
                        Log.Debug(Category.NetAdapter, "EtherType mapped from 0x{0:x4} to 0x{1:x4}", (ushort)raw.Type, perqType);
                        raw.Type = (EthernetType)perqType;
                    }
                }
            }
            catch (PcapException ex)
            {
                Log.Warn(Category.NetAdapter, "(Pcap) Failed to receive packet: {0}", ex.Message);
                return;
            }
            catch (Exception ex)
            {
                Log.Warn(Category.Network, "Failed to receive packet: {0}", ex.Message);
                return;
            }

            //
            // Look for RARPs, which are pretty rare these days and will
            // almost certainly be PERQemu (or maybe QEMU :-) emulated hosts
            // broadcasting a greeting
            //
            ArpPacket rarp;

            try
            {
                rarp = raw.Extract<ArpPacket>();

                if (rarp != null)
                {
                    Log.Debug(Category.Network, "RARP {0} received from {1}",
                              rarp.Operation, rarp.TargetHardwareAddress);

                    // If the Op is a RequestReverse (new host coming online) or a
                    // ReplyReverse (from others responding after our Request sent)
                    // the payload contains a host+Perq pair that we should add or
                    // update in our table!
                    if (rarp.Operation == ArpOperation.RequestReverse ||
                        rarp.Operation == ArpOperation.ReplyReverse)
                    {
                        // Trust, but verify:
                        if (IsPerqPrefix(rarp.TargetHardwareAddress))
                        {
                            var seen = _nat.LookupPerq(rarp.TargetHardwareAddress);
                            if (seen == null)
                            {
                                // Woo!  Another Perqy came out to play!
                                seen = new NATEntry(rarp.SenderHardwareAddress, rarp.TargetHardwareAddress);
                                _nat.Add(seen);

                                if (rarp.Operation == ArpOperation.RequestReverse)
                                {
                                    // Since this is the first time we've heard from this
                                    // host, send a RARP reply, since it's unlikely anyone
                                    // still has an in.rarpd running these days? :-)
                                    SendReply(rarp);
                                }
                            }
                        }
                        else
                        {
                            Log.Warn(Category.Network, "RARP target isn't a PERQ?");
                        }

                        // I'm pretty sure we can safely drop these here and not
                        // pass them on to the PERQ, which almost certainly won't
                        // do RARP (even under Accent).  HOWEVER, Accent's "new"
                        // message server (in S6+) will do actual IP ARPs, so we
                        // don't want to get in the way of those.
                        Log.Debug(Category.Network, "Local RARP handling complete");
                        return;
                    }
                }
                // Definitely fall through here
            }
            catch (PcapException ex)
            {
                Log.Debug(Category.Network, "Failed to parse RARP packet: {0}", ex.Message);
                // No biggie, just continue?
            }

            //
            // Does the PERQ want this packet?
            //
            if (!_controller.WantReceive(raw.DestinationHardwareAddress))
            {
                _pktsIgnored++;
                return;
            }

            //
            // We've run the gauntlet and received the packet; if we can handle it
            // right away, pass it on through, otherwise deal with the pending queue
            //

            // Shortcut: is the receiver active and ready?
            if (_controller.CanReceive)
            {
                if (_pending.IsEmpty)
                {
                    _pktsRecvd++;
                    _controller.Receive(raw.Bytes);
                    return;
                }

                // Push the newest, then pop and process the oldest
                _pending.Enqueue(raw);
                _pktsQueued++;

                if (_pending.TryDequeue(out raw))
                {
                    _pktsRecvd++;
                    _controller.Receive(raw.Bytes);
                    return;
                }

                Log.Warn(Category.NetAdapter, "Failed to dequeue at CanReceive!  Count={0}", _pending.Count);
                return;
            }

            // Controller is busy or not receiving; check if the queue has room
            if (_pending.Count < MaxBacklog)
            {
                _pending.Enqueue(raw);
                _pktsQueued++;

                Log.Debug(Category.NetAdapter, "Queued for later, count now {0}", _pending.Count);
                return;
            }

            // Queue is full, so toss the oldest packet.  If the PERQ is
            // that far behind either the 'net is busy and it can't keep
            // up, or it isn't actively receiving and we don't want to
            // inundate it with old traffic if it comes back online
            _pending.Enqueue(raw);
            _pktsQueued++;
            Flush();
        }

        /// <summary>
        /// Check if there are packets queued up; the controller is feelin' frisky.
        /// </summary>
        public void CheckReceive()
        {
            if (!_pending.IsEmpty && _controller.CanReceive)
            {
                EthernetPacket packet;

                if (!_pending.TryDequeue(out packet))
                {
                    Log.Warn(Category.NetAdapter, "Failed to dequeue at CheckReceive! Count={0}", _pending.Count);
                    return;
                }

                _pktsRecvd++;
                _controller.Receive(packet.Bytes);
            }
        }

        /// <summary>
        /// Determine if a received frame includes the FCS bytes or not, since
        /// we have to have an accurate byte count for the PERQ DMA.
        /// </summary>
        /// <remarks>
        /// I have three test Macs all running High Sierra with differnet NICs:
        /// a Broadcom, an Intel and an NVidia.  PacketDotNet/SharpPcap doesn't
        /// consistently include or strip the FCS bytes, so we do a simple check
        /// here and cache the result.  The PERQ network stack will often blow
        /// chunks if you overrun the DMA buffer due to an incorrect byte count!
        /// </remarks>
        void ProbeFCS(EthernetPacket packet)
        {
            if (!_probed)
            {
                // Another oddity: if the Payload is null, don't use this packet
                // and wait for a valid one.  More empirical weirdness with Pcap?
                // We should not be seeing 42 byte (VLAN tagged) payloads on the
                // initial RARP requests?  :-/
                if (!packet.HasPayloadData) return;

                // Assume the FCS is present and assemble the CRC
                var len = packet.Bytes.Length - 4;
                var check = ((packet.Bytes[len + 0] << 24) |
                             (packet.Bytes[len + 1] << 16) |
                             (packet.Bytes[len + 2] << 8) |
                              packet.Bytes[len + 3]);
                Log.Debug(Category.NetAdapter, "Received CRC is {0:x8}", check);

                // Compute checksum assuming the FCS is present
                var crc = Crc32.Compute(packet.Bytes, 0, packet.Bytes.Length - 4);
                Log.Debug(Category.NetAdapter, "Computed CRC is {0:x8}", crc);

                // Do they match?
                _hasFCS = (check == crc);

                // Recompute, to verify against what the sender logged (uh, manually)
                if (check != crc)
                {
                    crc = Crc32.Compute(packet.Bytes, 0, packet.Bytes.Length);
                    Log.Debug(Category.NetAdapter, "Re-computed CRC is {0:x8}", crc);
                }

                _probed = true;
            }
#if DEBUG
            else
            {
                var crc = Crc32.Compute(packet.Bytes, 0, _hasFCS ? packet.Bytes.Length - 4 : packet.Bytes.Length);
                Log.Debug(Category.NetAdapter, "Computed CRC is {0:x8}", crc);
            }
#endif
        }

        /// <summary>
        /// Stop packet capture, detach the callback and shutdown this instance.
        /// </summary>
        public void Shutdown()
        {
            // Stop, disable, and free up the timer
            HighResolutionTimer.Unregister(_natRefreshTimer);

            try
            {
                _adapter.StopCapture();
            }
            catch (Exception e)
            {
                // Log, but throw away exceptions since we're shutting down...
                Log.Info(Category.NetAdapter, "Exception on shutdown (ignored): {0}", e.Message);
            }
            finally
            {
                _adapter.OnPacketArrival -= OnPacketArrival;
                _adapter.Close();

                _nat.Flush();

                Log.Info(Category.NetAdapter, "Adapter shutdown");
            }
        }

        /// <summary>
        /// Check on the NAT table around once per minute (real time) to age out
        /// or update entries and queue up pings for hosts we haven't heard from
        /// in a while.
        /// </summary>
        void DoNATRefresh(HRTimerElapsedEventArgs args)
        {
            // Update current status
            _nat.Refresh();

            // Periodically send our announcements, too (which might trigger
            // responses that update the state we just set :-)
            SendGreeting();
        }

        /// <summary>
        /// Map a PERQ-specific EtherType to something that will pass on an 802.3
        /// network on transmit, or back again on receive.  Returns the type code
        /// unmodified if not a known PERQ type.  This is hacky as all get out.
        /// </summary>
        ushort PortMap(ushort etherType)
        {
            if (etherType < 1536 || (etherType >= EtherTypeMask && etherType < EtherTypeMask + 1536))
            {
                return (ushort)(etherType ^ EtherTypeMask);
            }

            return etherType;
        }

        /// <summary>
        /// Return true if a given address is in the official 3RCC address block.
        /// </summary>
        public static bool IsPerqPrefix(PhysicalAddress addr)
        {
            var a = addr.GetAddressBytes();

            return (a[0] == 0x02 && a[1] == 0x1c && a[2] == 0x7c && a[3] <= 2);
        }

        /// <summary>
        /// Weed out the non-Ethernet interfaces.  On Mac/Mono everything shows
        /// up as plain Ethernet (and most of these will never appear) but let's
        /// be complete.  I find it vaguely hilarious that MS includes 3Mbit as
        /// an enumeration.  That port of Windows to Alto, PDP-11 or PERQ coming
        /// along any day now?
        /// 
        /// This will probably go away since I'll probably have to use the #Pcap
        /// names to make it simpler to store/match names.  Ugh.
        /// </summary>
        public static bool IsEthernet(NetworkInterfaceType t)
        {
            return (t == NetworkInterfaceType.Ethernet ||
                    t == NetworkInterfaceType.Ethernet3Megabit ||
                    t == NetworkInterfaceType.FastEthernetFx ||
                    t == NetworkInterfaceType.FastEthernetT ||
                    t == NetworkInterfaceType.GigabitEthernet);
        }

        /// <summary>
        /// Find the adapter that matches the interface name.
        /// </summary>
        /// <remarks>
        /// The C# runtime gives back completely different names than the list
        /// SharpPcap (or its underlying LibPcap/WinPcap/AirPcap library) gives
        /// back, so here we map names based on platform type.
        /// 
        /// On Windows:
        ///     adapter.Id ~= dev.Name, without the rpcap:\\blah
        ///     adapter.Name can be assigned, is typ "Ethernet", "Ethernet 2", etc.
        ///     dev.Description has extra crap added by SharpPcap
        /// On Linux, Mac:
        ///     adapter.Id == dev.Name == adapter.Name == adapter.Description
        ///     dev.Description is blank; SharpPcap can't/doesn't get that info
        /// 
        /// To reconcile the two lists, we do OS-specific matching.  It's not
        /// pretty, but better than before.
        /// </remarks>
        public static ICaptureDevice GetAdapter(string name)
        {
            var interfaces = NetworkInterface.GetAllNetworkInterfaces();
            var devices = CaptureDeviceList.Instance;

            // Run through the list and try to match exactly...
            if (devices.Count > 0)
            {
                foreach (var dev in devices)
                {
                    if (PERQemu.HostIsUnix)
                    {
                        // The runtime name should match the SharpPcap name exactly
                        if (dev.Name.ToLowerInvariant() == name.ToLowerInvariant())
                            return dev;
                    }
                    else
                    {
                        foreach (var intf in interfaces)
                        {
                            // Use the runtime name find the interface, then loosely
                            // match the Ids
                            if (intf.Name.ToLowerInvariant() == name.ToLowerInvariant() &&
                                dev.Name.EndsWith(intf.Id, StringComparison.Ordinal))
                                return dev;
                        }
                    }
                }
            }

            Log.Info(Category.NetAdapter, "Could not find a match for Ethernet adapter '{0}'", name);
            return null;
        }

        /// <summary>
        /// Display the available host Ethernet interfaces.
        /// </summary>
        public static void ShowInterfaceSummary()
        {
            // Get the C# runtime's interface list
            var interfaces = NetworkInterface.GetAllNetworkInterfaces();

            foreach (NetworkInterface adapter in interfaces)
            {
                if (!IsEthernet(adapter.NetworkInterfaceType)) continue;

                Console.WriteLine($"ID: {adapter.Id}  Name: {adapter.Name}");
                if (adapter.Description != adapter.Name)
                    Console.WriteLine(adapter.Description);
                Console.WriteLine(string.Empty.PadLeft(adapter.Description.Length, '='));
                Console.WriteLine($"  Interface type:     {adapter.NetworkInterfaceType}");
                Console.WriteLine($"  Operational status: {adapter.OperationalStatus}");
                Console.WriteLine($"  Hardware address:   {adapter.GetPhysicalAddress()}");

                // Find and print the matching SharpPcap device
                var dev = GetAdapter(adapter.Name);
                Console.WriteLine("  SharpPcap device:   {0}", dev != null ? dev.Name : "[Not found!]");
                Console.WriteLine();
            }
        }


        // Debugging
        public void DumpStatus()
        {
            Console.WriteLine("\nHost adapter status:");
            Console.WriteLine($"  NIC: {Name} - {Description}");

            if (_probed)
                Console.WriteLine("  [This NIC {0} include FCS bytes in the payload]",
                                  _hasFCS ? "DOES" : "does NOT");
            else
                Console.WriteLine("  [FCS payload check not yet performed]");

            Console.WriteLine($"  Address: {Address}\tRunning: {Running}\tPending: {_pending.Count}");

            Console.WriteLine("\nInterface statistics:");
            Console.WriteLine($"  Total sent: {_pktsSent}\tReceived: {_pktsRecvd}\tIgnored: {_pktsIgnored}");
            Console.WriteLine($"  Deferred:   {_pktsQueued}\tDropped: {_pktsDropped}");

            _nat.DumpTable();
        }


        // Mask for mapping PERQ EtherType codes that fall within the Ethernet II
        // Length range (0..1535) to an unused range and back again.  (The offset
        // is chosen from unassigned space that IANA hasn't officially allocated.)
        const ushort EtherTypeMask = 0xb000;

        // All ones layer 2 broadcast
        public static PhysicalAddress Broadcast = new PhysicalAddress(new byte[] { 255, 255, 255, 255, 255, 255 });

        ICaptureDevice _adapter;
        INetworkController _controller;

        NATTable _nat;
        int _natRefreshTimer;

        public const int GreetingInterval = 15;     // Minimum, in seconds
        DateTime _lastGreeting = DateTime.Today;

        const int MaxBacklog = 15;                  // Don't queue without bound
        ConcurrentQueue<EthernetPacket> _pending;

        bool _hasFCS;                               // FCS bytes in received packets?
        bool _probed;

        ulong _pktsRecvd, _pktsSent;                // Some basic statistics,
        ulong _pktsQueued, _pktsDropped;            // for debugging/curiosity
        ulong _pktsIgnored;
    }
}

/*
    Notes:

    Old PERQ network software used EtherType codes in the 0..1535 range, which
    collide with the Length field in Ethernet II/IEEE 802.3 framing.  We map
    these so that virtual PERQs talking to each other see the EtherType values
    they expect, but the packets aren't misinterpreted on modern networks (they
    just appear as unknown/unassigned types in tcpdumps).
    
    Ethernet Type codes defined in E10Types.Pas (plus mapped equivalents):

    public static ushort[] PerqEtherTypes =
    {
		0x0000, EtherTypeMask,              // FTPByteStreamType
        0x0001, EtherTypeMask + 1,          // FTPEtherType
        0x0006, EtherTypeMask + 6,          // EchoServerType
        0x0007, EtherTypeMask + 7,          // TimeServerType
        0x0008, EtherTypeMask + 8,          // ServerRequest
        0x0090, EtherTypeMask + 144,        // Accent ConfigTest
        0x00db, EtherTypeMask + 219,        // Accent Time/repeater discovery?
        0x013b, EtherTypeMask + 315,        // CSDXServerType
        0x01c0, EtherTypeMask + 448,        // Accent EchoMe
        0x01c1, EtherTypeMask + 449         // Accent IAmAnEcho
    };

    PUP and the PUP "Addr Tran" (not explicitly noted in Accent?) are
    problematic; they should be reassigned to their relocated assigned
    numbers 0x0a00 and 0x0a01, but check with ContrAlto to see what if
    any remapping goes on there?  May need special handling here.
        0x0200, EtherTypeMask + 512,        // PUP
        0x0201, EtherTypeMask + 513         // PUP Addr Trans

    But doesn't 0x200 collide with Echo?  And what about 0x60 (Loop)?

    This is still based on the host-in-promiscuous-mode "raw" packet mode of the
    first implementation.  To add encapsulation options (wrapping PERQ traffic
    in UDP datagrams to avoid running PERQemu as root, for example) will require
    a different approach.  Plus, I want to move as much packet processing out of
    the receive event handler as possible to reduce possible concurrency issues,
    add new control/debug/visibility functions, etc.
    
*/
