﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PERQemu.Debugger
{
    /// <summary>
    /// Provides rudimentary debug support; at this time this is primarily mapping addresses
    /// to lines in corresponding source listings.
    /// </summary>
    public class Z80Debugger
    {
        public Z80Debugger()
        {
            LoadZ80Source(Paths.BuildPROMPath("pz80.lst"));
        }

        public string GetSourceLineForAddress(ushort address)
        {
            if (_sourceMap.ContainsKey(address))
            {
                return _sourceMap[address];
            }
            else
            {
                return null;
            }
        }

        public ushort GetAddressForSymbol(string symbol)
        {
            if (_symbolToAddressMap.ContainsKey(symbol))
            {
                return _symbolToAddressMap[symbol];
            }
            else
            {
                return 0;
            }
        }

        public string GetSymbolForAddress(ushort address, out ushort offset)
        {
            if (_addressToSymbolMap.ContainsKey(address))
            {
                offset = 0;
                return _addressToSymbolMap[address];
            }
            
            // Find the symbol nearest to the address
            // This could be done more efficiently.
            List<ushort> sortedAddresses = _addressToSymbolMap.Keys.ToList();
            sortedAddresses.Sort();
            for (int i = 0; i < sortedAddresses.Count; i++)
            {
                if (sortedAddresses[i] > address)
                {
                    if (i == 0)
                    {
                        // Nearest symbol is the start of the code.
                        offset = address;
                        return "<origin>";
                    }
                    else
                    {
                        // Nearest symbol is the previous one we looked at
                        offset = (ushort)(address - sortedAddresses[i - 1]);
                        return _addressToSymbolMap[sortedAddresses[i - 1]];
                    }
                }
            }

            // Nearest symbol is the last
            ushort addr = sortedAddresses[sortedAddresses.Count - 1];
            offset = (ushort)(address - addr);
            return _addressToSymbolMap[addr];
        }

        private void LoadZ80Source(string sourceFile)
        {
            _addressToSymbolMap = new Dictionary<ushort, string>();
            _symbolToAddressMap = new Dictionary<string, ushort>();
            _sourceMap = new Dictionary<ushort, string>();

            using (StreamReader sw = new StreamReader(sourceFile))
            {
                // Read the source listing in; scrape for symbol names, etc.
                while (!sw.EndOfStream)
                {
                    // Each line looks like:
                    //     ROM:addr <source code line>
                    // And each source code line may begin with a symbol (i.e. "PRQVEC:") or indented code.
                    // There should be no badly-formed lines as the listing is machine-generated.
                    // We'll try to be careful anyway.
                    string line = sw.ReadLine();

                    if (string.IsNullOrWhiteSpace(line))
                    {
                        throw new InvalidOperationException("Unexpected empty line in Z80 source listing.");
                    }

                    ushort address = ushort.Parse(line.Substring(4, 4), System.Globalization.NumberStyles.HexNumber);
                    string source = line.Substring(8);

                    // Index the source.
                    _sourceMap[address] = source;

                    // See if this line begins with a symbol and if so index it.
                    string[] tokens = source.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);

                    if (tokens.Length < 1)
                    {
                        continue;
                    }

                    if (tokens[0].EndsWith(":"))
                    {
                        string symbol = tokens[0].Substring(0, tokens[0].Length - 1);

                        _symbolToAddressMap[symbol] = address;
                        _addressToSymbolMap[address] = symbol;
                    }
                }
            }
        }

        private Dictionary<string, ushort> _symbolToAddressMap;
        private Dictionary<ushort, string> _addressToSymbolMap;
        private Dictionary<ushort, string> _sourceMap;

    }
}
