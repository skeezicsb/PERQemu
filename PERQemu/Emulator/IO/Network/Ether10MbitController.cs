//
// Ether10MbitController.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.Net.NetworkInformation;

using PERQemu.Config;

namespace PERQemu.IO.Network
{
    /// <summary>
    /// PERQ side of the "homegrown" OIO or EIO 10Mbit Ethernet controller.
    /// </summary>
    /// <remarks>
    /// Some programming documentation in Docs/HW/Ethernet_Guide_Sep81.txt and
    /// much more info about this implementation in Docs/Network.txt.
    /// </remarks>
    public class Ether10MbitController : NullEthernet
    {
        public Ether10MbitController(PERQSystem sys) : base(sys)
        {
            _nic = null;

            try
            {
                // Open the host network adapter
                _nic = new HostAdapter(this, Settings.EtherDevice);
            }
            catch (Exception e)
            {
                // In Debug, log the exception (likely a SharpPcap.PcapException)
                Log.Debug(Category.NetAdapter, "{0}", e.Message);

                throw new UnimplementedHardwareException("Could not open Ethernet device: adapter not found or no permissions");
            }
        }

        /// <summary>
        /// Controller hardware reset.
        /// </summary>
        public override void Reset()
        {
            _nic.Reset();
            base.Reset();
        }

        public override void Shutdown()
        {
            _nic.Shutdown();
            base.Shutdown();
        }

        /// <summary>
        /// Are we in the mood to handle another packet?
        /// </summary>
        public override bool CanReceive => _state == State.ReceiveWait;

        /// <summary>
        /// Let the adapter know we're interested in a given packet.  This is a
        /// simple filter to tell the hardware to give us the data or drop the
        /// packet based solely on destination address; the PERQ can run many
        /// different protocols so we don't do any interpretation at all.
        /// </summary>
        public override bool WantReceive(PhysicalAddress dest)
        {
            // Gimme gimme gimme
            if (_control.HasFlag(Control.Promiscuous)) return true;

            // Always accept L2 broadcasts, too
            if (dest.Equals(HostAdapter.Broadcast)) return true;

            // See if it's our hardware addr or current receive addr
            if (dest.Equals(_physAddr.PA)) return true;
            if (dest.Equals(_recvAddr.PA)) return true;

            // Finally, loop through the multicast bytes
            // (See Docs/Network.txt for more information!)
            var addr = dest.GetAddressBytes();
            if (addr[0] == 1 &&
                addr[1] == 0 &&
                addr[2] == 0 &&
                addr[3] == 0 &&
                addr[4] == 0 &&
                addr[5] != 0)
            {
                // No groups for you!
                if (MCB == 0xff) return false;

                // Receive all?
                if (MCB == 0) return true;

                // Match any five specific groups
                if (addr[5] == _mcastGroups[1] ||
                    addr[5] == _mcastGroups[2] ||
                    addr[5] == _mcastGroups[3] ||
                    addr[5] == _mcastGroups[4] ||
                    addr[5] == _mcastGroups[5])
                    return true;
            }

            // Otherwise log the rejection
            Log.Debug(Category.Ethernet, "Rejecting packet for {0}", dest);
            return false;
        }

        /// <summary>
        /// The microcode has put us in receive mode.  Check if anything is in
        /// the NIC's queue, which will kick off a Receive() if so.
        /// </summary>
        protected override void DoReceive()
        {
            _nic.CheckReceive();
        }

        /// <summary>
        /// Handle reception of a real live incoming packet!
        /// </summary>
        public override void Receive(byte[] packet)
        {
            // We must be actively receiving!  There's an obvious race here, if
            // we don't make _state volatile or work out a MUCH saner way to have
            // the controller *pull* packets from the NIC when ready, not have it
            // push them at us.  Either strategy could lead to unsightly gaps or
            // delays when there's a single packet in the queue and we miss the
            // window between going into receive mode and the next packet arriving
            // to prod the queue.  This is mostly to make sure we don't have a
            // concurrency issue where the bit counter is getting overwritten!
            if (_state != State.ReceiveWait)
            {
                Log.Warn(Category.Ethernet, "DoReceive in state {0}, packet dropped", _state);
                return;
            }

            // Update our state and status flag
            _state = State.Receiving;
            _status |= (Status.CarrierSense | Status.PacketInProgress);

            // Fetch the header and data buffer addresses from the DMAC
            var header = _system.IOB.DMARegisters.GetHeaderAddress(_dmaRx);
            var buffer = _system.IOB.DMARegisters.GetDataAddress(_dmaRx);

            ushort data;

            // Adjust our size in case the frame contains the FCS bytes
            var size = _nic.FrameIncludesFCS ? packet.Length - 4 : packet.Length;

            Log.Debug(Category.Ethernet, "Copying {0} bytes to header @ 0x{1:x6}, data @ 0x{2:x6} [{3}]",
                                         size, header, buffer,
                                         System.Threading.Thread.CurrentThread.ManagedThreadId);
            Log.Detail(Category.Ethernet, "Receive bit count initial = {0:x} ({1})",
                                        _bitCount, (short)_bitCount);

            // Write the frame's header to PERQ memory.  The header is 14 bytes
            // but the DMA always ships quad words; the first word is skipped/zero
            _system.Memory.StoreWord(header++, 0);

            for (var i = 0; i < 12; i += 2)
            {
                data = (ushort)(packet[i + 1] << 8 | packet[i]);
                _system.Memory.StoreWord(header++, data);
            }

            // Undo the length/type field swap that the software does
            data = (ushort)(packet[12] << 8 | packet[13]);
            _system.Memory.StoreWord(header, data);

            // DMA the packet data to the PERQ, reconstituted as 16-bit words
            for (var i = 14; i < size; i += 2)
            {
                data = packet[i];
                if (i + 1 < size) data |= (ushort)(packet[i + 1] << 8);
                _system.Memory.StoreWord(buffer++, data);
            }

            // Set the bit count as if the hardware had counted UP from the value
            // the microcode programmed; on receives the counter is intialized to
            // the 2's complement of 1518 (max frame size) because they use that
            // to detect giant packets.  Bit count INCLUDES the FCS bytes!
            _bitCount += (ushort)((size + 4) * 8);

            // Compute delay for DMA copy and schedule the callback to complete;
            // include the "interpacket gap" so we can do back-to-back receives
            var delay = (ulong)(((size + 4) * 8 * .1) + 9.6) * Conversion.UsecToNsec;
            _response = _system.Scheduler.Schedule(delay, ReceiveComplete);

            Log.Debug(Category.Ethernet, "Received {0} bytes ({1} bits), callback in {2}usec",
                                         size, (short)_bitCount, delay / 1000);
        }

        /// <summary>
        /// Transmit a packet, then schedule the callback to finish up.
        /// </summary>
        protected override void DoTransmit()
        {
            if (_bitCount > 0)
            {
                // Buffer (in bytes) for the complete raw packet, including header
                // but NOT the FCS -- the hardware adds the CRC32 on send!  (In
                // this case, SharpPcap/host adapter adds it on our behalf)
                byte[] packet = new byte[(_bitCount / 8)];

                // Get the addresses from the DMAC for the transmit channel
                var header = _system.IOB.DMARegisters.GetHeaderAddress(_dmaTx) + 1;
                var buffer = _system.IOB.DMARegisters.GetDataAddress(_dmaTx);
                ushort data;

                Log.Debug(Category.Ethernet, "Copying {0} bytes from header @ 0x{1:x6}, data @ 0x{2:x6} [{3}]",
                                             packet.Length, header, buffer,
                                             System.Threading.Thread.CurrentThread.ManagedThreadId);
                Log.Detail(Category.Ethernet, "Transmit bit count initial = {0:x} ({1})",
                                            _bitCount, (short)_bitCount);

                // DMA the header buffer from the PERQ's memory.  The hardware
                // always transfers two quads for the header, but skips over the
                // first (unused) word.
                for (var i = 0; i < 12; i += 2)
                {
                    data = _system.Memory.FetchWord(header++);
                    packet[i] = (byte)data;
                    packet[i + 1] = (byte)(data >> 8);
                }

                // Do the header's Length/Type field (swapped!)
                data = _system.Memory.FetchWord(header);
                packet[12] = (byte)(data >> 8);
                packet[13] = (byte)data;

                // Now copy the packet's payload from the buffer address

                // NB: we always use the packet length, and disregard whether the
                // buffer is properly quad-word aligned!  This may in fact lead
                // to some very subtle bugs, but for now assume that if it worked
                // on the hardware, the programmer must have figured out how to
                // properly allocate, align and lock down their buffers according
                // to the PERQ's Ethernet programming model.  La la la la la :-)
                for (var i = 14; i < packet.Length; i += 2)
                {
                    data = _system.Memory.FetchWord(buffer++);
                    packet[i] = (byte)data;
                    if (i + 1 < packet.Length) packet[i + 1] = (byte)(data >> 8);
                }

                // Hand off the complete packet!  Don't bother with return status
                // since there's nothing we could get back from Pcap that would
                // be meaningful to the microcode anyway :-(
                _nic.SendPacket(packet);
            }

            // Let the base method complete and log the transmission
            base.DoTransmit();
        }


        // Debugging
        public override void DumpEther()
        {
            base.DumpEther();

            if (_nic != null)
            {
                _nic.DumpStatus();
            }
        }

        // Handle for the host interface        
        HostAdapter _nic;
    }
}
