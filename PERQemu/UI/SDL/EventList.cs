//
// EventList.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using SDL2;

using System;
using System.Collections.Generic;

namespace PERQemu.UI
{
    /// <summary>
    /// Custom SDL2 events for PERQemu.
    /// </summary>
    public enum CustomEventType : uint
    {
        RenderPERQDisplay = 0,
        UpdateFPSDisplay,
        AudioIdleCheck
    }


    /// <summary>
    /// SDL2 message delegate for wiring standard and custom event callbacks.
    /// </summary>
    public delegate void SDLMessageHandlerDelegate(SDL.SDL_Event e);


    /// <summary>
    /// Consolidate all the custom SDL events and the dispatch list.
    /// </summary>
    public sealed class EventList
    {
        public EventList()
        {
            _dispatch = new Dictionary<SDL.SDL_EventType, SDLMessageHandlerDelegate>();

            _baseId = SDL.SDL_EventType.SDL_USEREVENT;
            _numIds = 0;
        }

        #region Register, release and invoke delegates

        /// <summary>
        /// Attach a delegate for an SDL event.
        /// </summary>
        public void Register(SDL.SDL_EventType e, SDLMessageHandlerDelegate d)
        {
            if (d == null)
                throw new InvalidOperationException("Can't register null delegate");

            if (_dispatch.ContainsKey(e))
                throw new InvalidOperationException($"Delegate already registered for event type {e}");

            _dispatch.Add(e, d);
            Log.Detail(Category.UI, "Attached delegate for SDL event type {0}", e);
        }

        /// <summary>
        /// Flush any pending, then release a delegate for an SDL event.  
        /// </summary>
        public void Release(SDL.SDL_EventType e)
        {
            if (_dispatch.ContainsKey(e))
            {
                SDL.SDL_FlushEvents(e, e);

                if (!_dispatch.Remove(e))
                    throw new InvalidOperationException($"Failed to release delegate for SDL event type {e}");

                Log.Detail(Category.UI, "Released delegate for SDL event type {0}", e);
            }
        }

        /// <summary>
        /// Invokes the delegate for a given event if it's in our list.
        /// </summary>
        public bool InvokeHandlerFor(SDL.SDL_Event e)
        {
            if (_dispatch.ContainsKey(e.type))
            {
                _dispatch[e.type].Invoke(e);
                return true;
            }

            return false;
        }

        #endregion

        #region Manage custom delegate IDs

        /// <summary>
        /// Allocates the custom event IDs if they haven't already been set up.
        /// The SDL2 API for this will make your eyes bleed.  Hope SDL3 is better?
        /// </summary>
        public void AllocateCustomEvents()
        {
            if (_numIds == 0)
            {
                // How many did we define?
                int howMany = Enum.GetValues(typeof(CustomEventType)).Length;

                // Allocate a block from SDL and save the base
                _baseId = (SDL.SDL_EventType)SDL.SDL_RegisterEvents(howMany);

                _numIds = (uint)howMany;

                Log.Info(Category.UI, "Allocated {0} SDL events, base = {1}",
                                      _numIds, _baseId);
            }
        }

        /// <summary>
        /// Assigns our custom identifier and user code to an SDL2 event.
        /// </summary>
        public void AssignEventID(ref SDL.SDL_Event e, CustomEventType t)
        {
            var code = (uint)t;

            // Lazily allocate 'em?  This won't bite me in the ass, right?
            if (_numIds == 0)
            {
                AllocateCustomEvents();
            }

            if (code > _numIds)
                throw new InvalidOperationException($"CustomEventType {t}");

            // Set the type and user code
            e.type = _baseId + code;
            e.user.code = (int)code;

            Log.Info(Category.UI, "Assigned ID {0} to {1} event", e.type, t);
        }

        /// <summary>
        /// Clear remnants of any lingering delegates and reset.
        /// </summary>
        public void Shutdown()
        {
            if (_dispatch.Count > 0)
            {
                // No, Artoo, shut them all down!
                SDL.SDL_FlushEvents(_baseId, _baseId + _numIds);
                _dispatch.Clear();

                Log.Info(Category.UI, "Custom events flushed");
            }

            _baseId = SDL.SDL_EventType.SDL_USEREVENT;
            _numIds = 0;
        }

        #endregion

        uint _numIds;
        SDL.SDL_EventType _baseId;

        Dictionary<SDL.SDL_EventType, SDLMessageHandlerDelegate> _dispatch;
    }
}
