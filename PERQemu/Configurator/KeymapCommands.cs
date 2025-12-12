//
// KeymapCommands.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using SDL2;

using PERQemu.Config;

namespace PERQemu.UI
{
    /// <summary>
    /// Commands for creating, editing and saving custom keyboard maps.
    /// </summary>
    public class KeymapCommands
    {
        [Command("keymap", "Enter the keyboard mapping subsystem", Prefix = true)]
        public void SetKeymapPrefix()
        {
            PERQemu.CLI.SetPrefix("keymap", PERQemu.Keymaps.Changed);
        }

        [Command("keymap done", "Exit storage configuration")]
        public void KeymapDone()
        {
            PERQemu.CLI.ResetPrefix();

            if (PERQemu.Keymaps.Changed())
            {
                Console.WriteLine("Note:  the current keyboard map has been modified but not saved.");
                Console.WriteLine("Use the 'keymap save' command to save your changes.");
            }
        }

        [Command("keymap commands", "Show keymap commands")]
        public void ShowKeymapCommands()
        {
            PERQemu.CLI.ShowCommands("keymap");
        }


        #region List and show commands

        [Command("keymap list", "List available saved keyboard maps")]
        public void ListKeymaps()
        {
            var maps = PERQemu.Keymaps.GetMapNames();
            Array.Sort(maps);

            if (maps.Length == 0)
            {
                Console.WriteLine("No keyboard maps loaded.");
                return;
            }

            Console.WriteLine("Defined keyboard maps:");

            for (var i = 0; i < maps.Length; i++)
            {
                var map = PERQemu.Keymaps.GetMapByName(maps[i]);
                Console.WriteLine("    {0} - {1}", map.Name.PadLeft(10), map.Description);
            }

            if (PERQemu.Keymaps.Current != null)
            {
                Console.WriteLine("Current keyboard map:");
                Console.WriteLine("    {0} - {1}", PERQemu.Keymaps.Current.Name.PadLeft(10),
                                  PERQemu.Keymaps.Current.Description);
            }
        }

        [Command("keymap show running", "Display active host->PERQ keyboard map")]
        public void ShowRunning()
        {
            if (PERQemu.Sys == null)
            {
                Console.WriteLine("No PERQ defined, can't show the active keyboard map.");
                return;
            }

            PERQemu.Keymaps.PrintMap(PERQemu.Sys.HID.Keyboard);
        }

        [Command("keymap show summary", "Summarize the curently loaded map")]
        [Command("keymap editor show summary", "Summarize the map currently being edited")]
        public void ShowSummary()
        {
            if (PERQemu.Keymaps.Current == null)
            {
                Console.WriteLine("No keyboard map currently loaded.");
                return;
            }

            Show(PERQemu.Keymaps.Current, true);    // Summarize
        }

        [Command("keymap show", "Display a saved keyboard map")]
        public void ShowKeymap([KeywordMatch("Keymaps")] string name)
        {
            var map = PERQemu.Keymaps.GetMapByName(name);

            if (map == null)
            {
                Console.WriteLine($"Can't find a map named '{name}'.");
                return;
            }

            Show(map);
        }

        [Command("keymap show", "Display the map currently loaded")]
        [Command("keymap editor show", "Display the map currently being edited")]
        public void ShowCurrent()
        {
            if (PERQemu.Keymaps.Current == null)
            {
                Console.WriteLine("No keyboard map currently loaded.");
                return;
            }

            Show(PERQemu.Keymaps.Current);
        }

        // Internal
        void Show(KeymapFile map, bool summarize = false)
        {
            Console.WriteLine($"Keymap name: {map.Name}");

            if (map.Filename != string.Empty)
                Console.WriteLine($"Filename:    {map.Filename}");

            if (map.Description != string.Empty)
                Console.WriteLine($"Description: {map.Description}");

            if (summarize)
                PERQemu.Keymaps.PrintSummary(map.Map);
            else
                PERQemu.Keymaps.PrintMap(map.Map);
        }

        #endregion

        #region Load, define and edit commands

        [Command("keymap load", "Load a saved keyboard map")]
        public bool LoadKeymap([KeywordMatch("Keymaps")] string name)
        {
            var map = PERQemu.Keymaps.GetMapByName(name);

            if (map == null)
            {
                Console.WriteLine($"Can't find a map named '{name}' to load.");
                return false;
            }

            PERQemu.Keymaps.SetCurrent(map);
            return true;
        }

        [Command("keymap define", "Initialize and edit a new keyboard map")]
        public void DefineKeymap(string name)
        {
            // Name reserved or already in use?
            if (!SetName(name)) return;

            // Set up the default map and go!
            PERQemu.Keymaps.DefineMap(name);
            StartEditing();
        }

        [Command("keymap edit", "Load and edit a saved keyboard map")]
        public void EditKeymap([KeywordMatch("Keymaps")] string name)
        {
            if (LoadKeymap(name))
                EditKeymap();
        }

        [Command("keymap edit", "Enter the keymap editor")]
        public void EditKeymap()
        {
            if (PERQemu.Keymaps.Current == null)
            {
                Console.WriteLine("No current keyboard map; please 'load' or 'define' one to edit.");
                return;
            }

            if (PERQemu.Keymaps.Current.Key == "default")
            {
                Console.WriteLine("Can't change the default keymap; please 'define' a new one");
                Console.WriteLine("or load a saved map for editing.");
                return;
            }

            StartEditing();
        }

        #endregion

        #region Apply and save commands

        [Command("keymap apply", "Apply the current keymap to the running PERQ")]
        [Command("keymap editor apply", "Apply the current keymap to the running PERQ")]
        public void ApplyMap()
        {
            if (PERQemu.Keymaps.Current == null)
            {
                Console.WriteLine("No map loaded!  No changes to apply.");
                return;
            }

            if (PERQemu.Sys == null || PERQemu.Controller.State == RunState.Off)
            {
                Console.WriteLine("The PERQ is not defined or running; cannot apply changes.");
                return;
            }

            PERQemu.Keymaps.Apply();
        }

        [Command("keymap save", "Save the current keyboard map")]
        [Command("keymap editor save", "Save the current keyboard map")]
        public void SaveMap()
        {
            // Sanity check
            if (string.IsNullOrEmpty(PERQemu.Keymaps.Current?.Filename))
            {
                Console.WriteLine("No map or filename not set!  Save failed.");
                return;
            }

            PERQemu.Keymaps.Save();
        }

        #endregion

        #region Editor commands

        [Command("keymap editor", Prefix = true, Discreet = true)]
        public void StartEditing()
        {
            if (PERQemu.Keymaps.Current == null) return;

            PERQemu.CLI.SetPrefix("keymap editor", PERQemu.Keymaps.Changed);
            ShowKeypresses(true);
        }

        [Command("keymap editor commands", "Show keymap editor commands")]
        public void ShowKeyEditCommands()
        {
            PERQemu.CLI.ShowCommands("keymap editor");
        }

        public bool SetName(string name)
        {
            var key = name.Trim().ToLower();

            if (key == "default" || key == "none")
            {
                Console.WriteLine($"The name '{key}' is reserved; please choose another.");
                return false;
            }

            if (PERQemu.Keymaps.GetMapByName(key) != null)
            {
                Console.WriteLine("There's already a keymap by that name; please choose another.");
                return false;
            }

            return true;
        }

        [Command("keymap editor default", "Reset all mappings to their default")]
        public void SetDefault()
        {
            PERQemu.Keymaps.ResetMap();
            Console.WriteLine("Keyboard map reset to defaults.");
        }

        [Command("keymap editor description", "Add a brief description of the keyboard map")]
        public void SetDescription(string desc)
        {
            PERQemu.Keymaps.SetDescription(desc);
        }

        [Command("keymap editor map", "Set or change the mapping of a key")]
        public void MapKey(SDL.SDL_Keycode hostKey, KeyCap perqKey)
        {
            if (PERQemu.Keymaps.IsReserved(hostKey))
            {
                Console.WriteLine($"Can't remap {hostKey} (reserved).");
                return;
            }

            if (hostKey == SDL.SDL_Keycode.SDLK_UNKNOWN)
            {
                // Special case: remove the default mappings for the
                // PERQ key... this is potentially confusing as hell
                var mapped = PERQemu.Keymaps.Current.Map.GetMappingsFor(perqKey);

                foreach (var key in mapped)
                {
                    UnmapKey(key);
                }
                return;
            }

            // Do a normal re-map, quietly if just starting up
            if (PERQemu.Keymaps.MapKey(hostKey, perqKey) && PERQemu.Initialized)
            {
                ShowHostKey(hostKey);
            }
        }

        [Command("keymap editor unmap", "Remove a key mapping")]
        public void UnmapKey(SDL.SDL_Keycode hostKey)
        {
            MapKey(hostKey, KeyCap.None);   // Lazy :-)
        }

        [Command("keymap editor show host key", "Show the mapping for a host key")]
        public void ShowHostKey(SDL.SDL_Keycode hostKey)
        {
            Console.Write($"Host key {hostKey} is ");

            if (PERQemu.Keymaps.IsReserved(hostKey))
            {
                Console.WriteLine("reserved by PERQemu.");
                return;
            }

            var key = PERQemu.Keymaps.Current.Map.GetKeyMapping(hostKey);

            if (key == KeyCap.None)
                Console.WriteLine("unmapped.");
            else
                Console.WriteLine($"mapped to {key}.");
        }

        [Command("keymap editor show mapped key", "Show what's mapped to a specific PERQ key")]
        public void ShowMappedKey(KeyCap perqKey)
        {
            var mapped = PERQemu.Keymaps.Current.Map.GetMappingsFor(perqKey);

            if (mapped.Count == 0)
            {
                Console.WriteLine($"There are no keys mapped to {perqKey}!");
                return;
            }

            foreach (var key in mapped)
            {
                Console.WriteLine($"Host key {key} is mapped to {perqKey}.");
            }
        }

        [Command("keymap editor done", "Return to the keymap subsystem")]
        public void DoneEditing()
        {
            PERQemu.Keymaps.AddOrUpdateMap();
            ShowKeypresses(false);
            SetKeymapPrefix();
        }

        void ShowKeypresses(bool enable)
        {
            if (PERQemu.Sys == null) return;

            if (enable != PERQemu.Sys.HID.ShowKeycodes)
            {
                PERQemu.Sys.HID.ShowKeycodes = enable;
                Console.WriteLine((enable ? "Showing" : "Not showing") + " key presses.");
            }
        }

        #endregion

    }
}
