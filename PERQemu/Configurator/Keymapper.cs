//
// Keymapper.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.IO;
using System.Collections.Generic;

using SDL2;

using PERQemu.IO;
using PERQemu.UI;

namespace PERQemu.Config
{
    /// <summary>
    /// In-memory representation of a keyboard mapping file, with methods to
    /// load and save 'em to the Conf directory.  Always based on the PERQ-2
    /// VT-100 style, a superset of both supported types.
    /// </summary>
    public class KeymapFile
    {
        public KeymapFile(string name)
        {
            _map = new KeyboardMap(KeyboardType.VT100);

            _description = string.Empty;
            _filename = string.Empty;

            Name = name;    // set key
        }

        public string Name
        {
            get { return _name; }
            set
            {
                _name = value;
                _key = _name.Trim().ToLower();
            }
        }

        public string Key => _key;

        public string Filename
        {
            get { return _filename; }
            set { _filename = value; }
        }

        public string Description
        {
            get { return _description; }
            set { _description = value; }
        }

        public KeyboardMap Map
        {
            get { return _map; }
            set { _map = value; }
        }

        string _name;           // short (file) name
        string _description;    // brief description
        string _filename;       // full saved filename
        string _key;            // hash key for matching
        KeyboardMap _map;       // actual SDL2->PERQ key maps
    }

    /// <summary>
    /// Like the Configurator, but for keyboard maps.
    /// </summary>
    public class Keymapper
    {
        public Keymapper()
        {
            _loadedMaps = new Dictionary<string, KeymapFile>();
            _reserved = new List<SDL.SDL_Keycode>();
            _current = null;
        }

        // Handle to one working copy
        public KeymapFile Current => _current;

        // Delegate for command prompt check
        public bool Changed()
        {
            return _modified || !_saved;
        }

        public void Initialize()
        {
            // Preload all the maps in Conf
            LoadKeymaps();

            // Set aside the reserved keys
            ReserveKeys();

            // Unload
            _current = null;
            _modified = false;
            _saved = true;
        }

        public bool IsReserved(SDL.SDL_Keycode key)
        {
            return _reserved.Contains(key);
        }

        void LoadKeymaps()
        {
            // Add the default (built-in)
            DefineMap("default");
            _current.Description = "Default (built-in) keyboard mapping";
            AddOrUpdateMap();

            Log.Debug(Category.MediaLoader, "Loading keyboard maps from '{0}'",
                                            Paths.Canonicalize(Paths.ConfigDir));

            foreach (var file in Directory.EnumerateFiles(Paths.ConfigDir, "*.kbd"))
            {
                if (Load(Paths.Canonicalize(file)))
                {
                    // Loading always updates _current on success
                    Log.Detail(Category.MediaLoader, "Added keyboard map '{0}'", _current.Name);
                }
            }
        }

        public bool Load(string path)
        {
            try
            {
                PERQemu.CLI.ReadScript(path);
                return true;
            }
            catch (Exception e)
            {
                Log.Write(Category.Keyboard, "Failed to load keymap from '{0}': {1}", path, e.Message);
                return false;
            }
        }

        public void Save()
        {
            // Don't rewrite if nothing has changed
            if (!Changed()) return;

            Console.Write($"Saving keyboard map {_current.Name} to {_current.Filename}...");

            try
            {
                using (StreamWriter sw = new StreamWriter(_current.Filename, false))
                {
                    sw.WriteLine($"# PERQemu keyboard mapping file, written {DateTime.Now}");
                    sw.WriteLine($"# {PERQemu.Version}");
                    sw.WriteLine("#");
                    sw.WriteLine("#    Please consult the PERQemu User's Guide for information about");
                    sw.WriteLine("#    editing this file.  Errors or duplicate keys will be ignored.");
                    sw.WriteLine("#");
                    sw.WriteLine("keymap");
                    sw.WriteLine($"define {_current.Name}");

                    if (!string.IsNullOrWhiteSpace(_current.Description))
                        sw.WriteLine($"description \"{_current.Description}\"");

                    // Cheeky way to get the last defined element?
                    var count = Enum.GetValues(typeof(KeyCap)).Length;
                    var last = (KeyCap)count - 1;

                    //
                    // Loop over the enumerated PERQ KeyCaps so we can write out
                    // a full template for external hand editing, mapped or not:
                    //   1. Keys that have a default mapping that are not
                    //      present are noted as unmapped;
                    //   2. Keys that aren't normally mapped to anything and
                    //      are mapped to None are quietly dropped.
                    //
                    for (var cap = KeyCap.None; cap < last; cap++)
                    {
                        // Add some comments to enhance readability
                        if (cap == KeyCap.A)
                            sw.WriteLine("\n# Common alphanumerics");
                        else if (cap == KeyCap.Help)
                            sw.WriteLine("\n# Special keys (common)");
                        else if (cap == KeyCap.Setup)
                            sw.WriteLine("\n# Special keys (PERQ-2 only)");
                        else if (cap == KeyCap.PadPlus)
                            sw.WriteLine("\n# Pseudo keys");

                        // Find SDL keys that point to this cap!
                        var mapped = _current.Map.GetMappingsFor(cap);

                        if (mapped.Count == 0)
                        {
                            if (cap != KeyCap.None)
                                sw.WriteLine($"# Nothing mapped to '{cap}'!");
                        }
                        else
                        {
                            // Many to one
                            foreach (var key in mapped)
                            {
                                if (cap == KeyCap.None)
                                    sw.WriteLine($"unmap {key}");
                                else
                                    sw.WriteLine($"map {key} {cap}");
                            }
                        }
                    }

                    sw.WriteLine("done");
                    sw.Close();
                }

                _modified = false;
                _saved = true;
                Console.WriteLine("done.");
            }
            catch (Exception e)
            {
                _saved = false;
                Console.WriteLine("failed!");
                Console.WriteLine(e.Message);
            }
        }

        public void Apply()
        {
            Apply(_current);
        }

        public void Apply(KeymapFile map)
        {
            // Sanity check
            if (map == null || PERQemu.Sys == null) return;

            var codes = PERQemu.Sys.Config.Keyboard == KeyboardType.PERQ ?
                               KeyboardCodes.PERQ1Codes : KeyboardCodes.PERQ2Codes;

            var changes = new Dictionary<SDL.SDL_Keycode, KeyCap>();

            var active = PERQemu.Sys.HID.Keyboard;

            // Check all the keys in the new map
            foreach (var k in map.Map.Keys)
            {
                var newCap = map.Map.GetKeyMapping(k);
                var oldCap = active.GetKeyMapping(k);

                // Is this relevant to this keyboard type?
                if (newCap != oldCap && codes[(int)newCap].Name != KeyCap.None)
                {
                    // Yes, update the mapping
                    changes.Add(k, newCap);
                }
            }

            // Loop through the active map to find deletions
            foreach (var k in active.Keys)
            {
                var newCap = map.Map.GetKeyMapping(k);
                var oldCap = active.GetKeyMapping(k);

                if (newCap == KeyCap.None && oldCap != KeyCap.None)
                {
                    changes.Add(k, KeyCap.None);
                }
            }

            // Now apply them (since we can't modify the active enumeration, above)
            foreach (var update in changes)
            {
                PERQemu.Sys.HID.Keyboard.SetKeyMapping(update.Key, update.Value);
            }
        }

        public void SetCurrent(KeymapFile map)
        {
            if (_current != map)
            {
                _current = map;
                _modified = false;
                _saved = true;
            }
        }

        public void DefineMap(string name)
        {
            _current = new KeymapFile(name);
            _current.Filename = Paths.QualifyPathname(_current.Key, Paths.ConfigDir, ".kbd", true);
            _modified = false;
            _saved = false;
        }

        public void ResetMap()
        {
            _current.Map = new KeyboardMap(KeyboardType.VT100);
            _modified = true;
        }

        public void SetDescription(string desc)
        {
            _current.Description = desc;
            _modified = true;
        }

        public void AddOrUpdateMap()
        {
            if (_loadedMaps.ContainsKey(_current.Key))
            {
                _loadedMaps[_current.Key] = _current;
            }
            else
            {
                _loadedMaps.Add(_current.Key, _current);
                PERQemu.CLI.UpdateKeywordMatchHelpers("Keymaps", GetMapNames());
            }
        }

        public KeymapFile GetMap(string key)
        {
            return _loadedMaps.ContainsKey(key) ? _loadedMaps[key] : null;
        }

        public KeymapFile GetMapByName(string name)
        {
            return GetMap(name.Trim().ToLower());
        }

        public string[] GetMapNames()
        {
            var maps = new string[_loadedMaps.Count];
            _loadedMaps.Keys.CopyTo(maps, 0);
            return maps;
        }

        public void PrintMap(KeyboardMap keymap)
        {
            if (keymap == null) return;

            var list = new List<string>();

            Console.WriteLine("Host key to PERQ key map:");
            foreach (var k in keymap.Keys)
            {
                list.Add($"{k.ToString()} => {keymap.GetKeyMapping(k)}");
            }

            PERQemu.CLI.Columnify(list.ToArray(), 4, 36);
        }

        public void PrintSummary(KeyboardMap keymap)
        {
            if (keymap == null) return;

            var std = _loadedMaps["default"].Map;

            var list = new List<string>();

            // Loop through the NEW map to find additions and changes
            foreach (var k in keymap.Keys)
            {
                var newCap = keymap.GetKeyMapping(k);
                var oldCap = std.GetKeyMapping(k);

                if (oldCap == KeyCap.None)
                {
                    list.Add($"    + Key {k.ToString()} is mapped to {newCap}");
                }
                else if (newCap != oldCap && newCap != KeyCap.None)
                {
                    list.Add($"    = Key {k.ToString()} remapped from {oldCap} to {newCap}");
                }
                // otherwise it's the same in both lists
            }

            // Loop through the STD map to find deletions
            foreach (var k in std.Keys)
            {
                var newCap = keymap.GetKeyMapping(k);
                var oldCap = std.GetKeyMapping(k);

                if (newCap == KeyCap.None)
                {
                    list.Add($"    - Key {k.ToString()} is unmapped (was {oldCap})");
                }
            }

            // if list is empty, no changes!
            if (list.Count == 0)
            {
                Console.WriteLine("No changes from the default map.");
                return;
            }

            Console.WriteLine("Differences:");
            foreach (var line in list)
            {
                Console.WriteLine(line);
            }
        }

        public bool MapKey(SDL.SDL_Keycode hostKey, KeyCap perqKey)
        {
            var currentKey = _current.Map.GetKeyMapping(hostKey);

            if (currentKey == perqKey) return false;

            // Change the map
            _current.Map.SetKeyMapping(hostKey, perqKey);
            _modified = true;
            return true;
        }

        /// <summary>
        /// Reserves the keys used by PERQemu so they can't be remapped.  This
        /// really doesn't belong here!  Make sure it agrees with InputDevices.
        /// All this should be refactored properly someday.  Oof.
        /// </summary>
        void ReserveKeys()
        {
            _reserved.Add(SDL.SDL_Keycode.SDLK_HOME);
            _reserved.Add(SDL.SDL_Keycode.SDLK_END);
            _reserved.Add(SDL.SDL_Keycode.SDLK_CAPSLOCK);
            _reserved.Add(SDL.SDL_Keycode.SDLK_NUMLOCKCLEAR);
            _reserved.Add(SDL.SDL_Keycode.SDLK_LSHIFT);
            _reserved.Add(SDL.SDL_Keycode.SDLK_RSHIFT);
            _reserved.Add(SDL.SDL_Keycode.SDLK_LCTRL);
            _reserved.Add(SDL.SDL_Keycode.SDLK_RCTRL);
            _reserved.Add(SDL.SDL_Keycode.SDLK_LALT);
            _reserved.Add(SDL.SDL_Keycode.SDLK_RALT);
            _reserved.Add(SDL.SDL_Keycode.SDLK_MENU);
            _reserved.Add(SDL.SDL_Keycode.SDLK_PAUSE);
            _reserved.Add(SDL.SDL_Keycode.SDLK_F8);
            _reserved.Add(SDL.SDL_Keycode.SDLK_PRINTSCREEN);
        }

        // All the preloaded/defined keymaps
        Dictionary<string, KeymapFile> _loadedMaps;

        // A list of reserved keys
        List<SDL.SDL_Keycode> _reserved;

        // Working map
        KeymapFile _current;

        bool _modified;
        bool _saved;
    }
}
