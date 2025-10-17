//
// Translation.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.Collections.Generic;
using System.Net.NetworkInformation;

namespace PERQemu.IO.Network
{
    public enum Status
    {
        Unknown = 0,
        Active,                 // An active peer
        Idle,                   // A peer that's gone quiet...
        Stale                   // An inactive or invalid entry
    }

    [Flags]
    public enum Flags           // Todo: rename/refactor/remove
    {
        None = 0x0,
        Me = 0x1,               // This host!
        EncapUDP = 0x20,        // Host is using UDP encapsulation
        Encap3in10 = 0x40,      // A 3Mbit host (UDP encapsulated)
        EncapNone = 0x80        // A REAL PERQ! (no translation)
    }

    /// <summary>
    /// Store an Ethernet address translation so that virtual PERQs can be easily
    /// mapped to their hosts.  Largely for stats/debugging?
    /// </summary>
    public class NATEntry
    {
        public NATEntry(PhysicalAddress host, PhysicalAddress perq, Flags flags = Flags.None)
        {
            Host = host;
            Perq = perq;
            FirstSeen = DateTime.Now;
            LastReceived = DateTime.Now;
            Flags = flags;
            State = Status.Active;
            Sent = Received = 0;
        }

        public override string ToString()
        {
            return $"[Host: {Host}  Perq: {Perq}  State: {State}]";
        }

        public void UpdateReceived()
        {
            LastReceived = DateTime.Now;
            State = Status.Active;
            Received++;
        }

        public PhysicalAddress Host;        // Host MAC
        public PhysicalAddress Perq;        // PERQ's MAC
        public DateTime FirstSeen;          // Date/time mapping established
        public DateTime LastReceived;       // Date/time last packet/update received
        public Status State;                // Track peer activity
        public Flags Flags;                 // Info about this entry
        public ulong Received;              // Count Host->Perq mappings
        public ulong Sent;                  // Count Perq->Host mappings
    }

    /// <summary>
    /// A (very) simple Network Address Translation table, mapping a given host
    /// MAC address to an emulated PERQ address.
    /// </summary>
    /// <remarks>
    /// This operates at the Ethernet layer and is only (currently) used to map
    /// PERQ-to-PERQ traffic on a local segment.  It does NOT yet allow for one
    /// host computer to run multiple PERQemu instances (which it may have to!)
    /// and it does NOT operate at the IP layer (which it might also, to allow
    /// "discovery" of other PERQs on other subnets or through a global registry
    /// of some kind.  See Docs/Network.txt for more information.
    /// </remarks>
    public class NATTable
    {
        public NATTable()
        {
            _entries = new Dictionary<PhysicalAddress, NATEntry>();
            _perqToHost = new Dictionary<PhysicalAddress, PhysicalAddress>();
        }

        public void Reset()
        {
        }

        /// <summary>
        /// Flush the entire table.
        /// </summary>
        public void Flush()
        {
            _entries.Clear();
            _perqToHost.Clear();
            Log.Info(Category.Network, "NAT table flushed");
        }

        /// <summary>
        /// Add a new mapping.  We leave initialization up to the caller.
        /// </summary>
        public bool Add(NATEntry ent)
        {
            // Make sure it isn't already there...
            if (_entries.ContainsKey(ent.Host))
            {
                // ...but only gripe if it's active :-)
                if (_entries[ent.Host].State == Status.Active)
                {
                    Log.Warn(Category.Network, "Duplicate NAT entry for active host, ignored {0}", ent);
                    return false;
                }

                // Clobber it and fall thru!
                Drop(ent);
            }

            // Check the inverse index too
            if (_perqToHost.ContainsKey(ent.Perq))
            {
                Log.Warn(Category.Network, "PERQ {0} already in index at different host?", ent.Perq);
                return false;
            }

            // Map it!
            _entries.Add(ent.Host, ent);
            _perqToHost.Add(ent.Perq, ent.Host);

            Log.Debug(Category.Network, "NAT entry added {0}", ent);
            return true;
        }

        /// <summary>
        /// Search the table to see if we've seen PERQ traffic from a given source
        /// MAC address before.  If so, return the NAT entry; otherwise, null.
        /// </summary>
        public NATEntry LookupHost(PhysicalAddress host)
        {
            return _entries.ContainsKey(host) ? _entries[host] : null;
        }

        /// <summary>
        /// Look up another PERQ (real or virtual) in the inverse map and return
        /// the host entry for it.
        /// </summary>
        public NATEntry LookupPerq(PhysicalAddress perq)
        {
            return _perqToHost.ContainsKey(perq) ? LookupHost(_perqToHost[perq]) : null;
        }

        /// <summary>
        /// Drop a mapping from the tables.
        /// </summary>
        public void Drop(NATEntry mapping)
        {
            if (_entries.ContainsKey(mapping.Host) && _perqToHost.ContainsKey(mapping.Perq))
            {
                if (!_perqToHost.Remove(mapping.Perq)) throw new InvalidOperationException($"Drop failed: Perq {mapping}");
                if (!_entries.Remove(mapping.Host)) throw new InvalidOperationException($"Drop failed: Host {mapping}");

                Log.Debug(Category.Network, "Dropped mapping {0}", mapping);
            }
        }

        /// <summary>
        /// Refresh the status of each peer entry based on elapsed time
        /// since any observed network activity from them.  There are plenty
        /// of obvious issues here (like, pausing emulation) but it's a start.
        /// </summary>
        public void Refresh()
        {
            const int IdleTime = 180;       // Three minutes?
            const int OfflineTime = 900;    // Fifteen minutes?

            foreach (var ent in _entries.Values)
            {
                if (ent.Flags.HasFlag(Flags.Me)) continue;

                TimeSpan ts = DateTime.Now - ent.LastReceived;
                var last = ts.TotalSeconds;

                if (last <= IdleTime)
                    ent.State = Status.Active;
                else if (last > IdleTime && last <= OfflineTime)
                    ent.State = Status.Idle;
                else
                    ent.State = Status.Stale;

                Log.Debug(Category.Network, "Refresh: {0}", ent);
            }
        }

        // Debugging
        public void DumpTable()
        {
            if (_entries.Count == 0)
            {
                Console.WriteLine("  NAT table is empty.");
                return;
            }

            // Make it purty:
            // 001122334455  001122334455  dd/mm/yyyy hh:mm:ss am  ssssss  rrrrrr
            // flags                       dd/mm/yyyy hh:mm:ss pm  age
            string fmt1 = "{0,-12}  {1,-12}  {2,-22}  {3,6}  {4,6}";
            string fmt2 = "{0,-26}  {1,-22}  {2,14}\n";

            Console.WriteLine("\nNAT table:\n");
            Console.WriteLine(fmt1, "Host /", "PERQ", "First seen /", "Sent", "Rcvd");
            Console.WriteLine(fmt2, "State", "Last received", "Age (hh:mm:ss)");

            foreach (var e in _entries.Values)
            {
                var age = (DateTime.Now - e.LastReceived).ToString(@"hh\:mm\:ss");
                var status = (e.Flags.HasFlag(Flags.Me) ? "Me!" : e.State.ToString());
                Console.WriteLine(fmt1, e.Host, e.Perq, e.FirstSeen, e.Sent, e.Received);
                Console.WriteLine(fmt2, status, e.LastReceived, age);
            }
        }

        Dictionary<PhysicalAddress, NATEntry> _entries;
        Dictionary<PhysicalAddress, PhysicalAddress> _perqToHost;
    }
}
