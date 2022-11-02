# dialog_cache_outdated.py - inform the user to update the apt cache
#  
#  Copyright (c) 2006 Canonical
#  
#  Authors: 
#       Sebastian Heinlein <sebastian.heinlein@web.de>
# 
#  This program is free software; you can redistribute it and/or 
#  modify it under the terms of the GNU General Public License as 
#  published by the Free Software Foundation; either version 2 of the
#  License, or (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
#  USA

import os
import gi
gi.require_version("Gdk", "3.0")
gi.require_version("Gtk", "3.0")
gi.require_version("PackageKitGlib", "1.0")
from gi.repository import GObject, Gdk, Gtk
from gi.repository import PackageKitGlib as packagekit
from gettext import gettext as _

from softwareproperties.gtk.utils import (
    setup_ui,
)


class ProgressDialog(Gtk.Window):
    """A small helper window to display progress"""

    def __init__(self, parent):
        Gtk.Window.__init__(self, title=_("Cache Refresh"))
        self.set_transient_for(parent)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.set_border_width(16)
        self.set_modal(True)
        self.set_deletable(False)

        self.set_default_size(300, 75)
        geometry = Gdk.Geometry()
        geometry.min_width = 210
        geometry.min_height = 60
        geometry.max_width = 800
        geometry.max_height = 260
        self.set_geometry_hints(None, geometry, Gdk.WindowHints.MIN_SIZE)
        self.set_geometry_hints(None, geometry, Gdk.WindowHints.MAX_SIZE)

        self.box = Gtk.Box(spacing=6, orientation=Gtk.Orientation.VERTICAL)
        self.add(self.box)

        self.label = Gtk.Label(xalign=0)
        self.label.set_markup("<b><big>{}</big></b>".format(_("Refreshing software cache")))
        self.box.pack_start(self.label, False, False, 0)

        # create a progress bar
        self.progressbar = Gtk.ProgressBar()
        self.box.pack_start(self.progressbar, True, True, 0)


class DialogCacheOutdated:
    def __init__(self, parent, datadir):
        """setup up the gtk dialog"""
        self.parent = parent

        setup_ui(self, os.path.join(datadir, "gtkbuilder", "dialog-cache-outofdate.ui"), domain="software-properties")
        self.dialog = self.dialog_cache_outofdate
        self.dialog.set_transient_for(parent)

    def on_pktask_progress(self, progress, ptype, udata=(None,)):
        if ptype == packagekit.ProgressType.PERCENTAGE:
            perc = progress.get_property('percentage')
            self._pdia.progressbar.set_fraction(perc / 100.0)

    def on_pktask_finish(self, source, result, udata=(None,)):
        results = None
        try:
            results = self._pktask.generic_finish(result)
        except Exception as e:
            dialog = Gtk.MessageDialog(self.parent, 0, Gtk.MessageType.ERROR,
                Gtk.ButtonsType.CANCEL, _("Error while refreshing cache"))
            dialog.format_secondary_text(str(e))
            dialog.run()
        self._loop.quit ()

    def run(self):
        """run the dialog, and if reload was pressed run cache update"""
        res = self.dialog.run()
        self.dialog.hide()
        if res == Gtk.ResponseType.APPLY:
            self._pktask = packagekit.Task()
            self._pdia = ProgressDialog(self.parent)
            self._loop = GObject.MainLoop()
            self._pdia.show_all()

            self.parent.set_sensitive(False)
            try:
                self._pktask.refresh_cache_async (False, # force
                                                  None,  # GCancellable
                                                  self.on_pktask_progress,
                                                  (None,), # user data
                                                  self.on_pktask_finish,
                                                  (None,));
            except Exception as e:
                print("Error while requesting cache refresh: {}".format(e))

            self._loop.run()
            self._pdia.hide()
            self.parent.set_sensitive(True)

        return res
