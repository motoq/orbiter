/*
 c  CreditsJPanel.java
 c
 c  Copyright (C) 2012 Kurt Motekew
 c
 c  This library is free software; you can redistribute it and/or
 c  modify it under the terms of the GNU Lesser General Public
 c  License as published by the Free Software Foundation; either
 c  version 2.1 of the License, or (at your option) any later version.
 c
 c  This library is distributed in the hope that it will be useful,
 c  but WITHOUT ANY WARRANTY; without even the implied warranty of
 c  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 c  Lesser General Public License for more details.
 c
 c  You should have received a copy of the GNU Lesser General Public
 c  License along with this library; if not, write to the Free Software
 c  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 c  02110-1301 USA
 */

package com.motekew.orbiter.gui;

import javax.swing.*;

/*
 * Creates a JPanel with text listing external resources
 * and other information.
 *
 * @author   Kurt Motekew
 * @since    20120929
 */
public class CreditsJPanel extends JPanel {
  public CreditsJPanel() {
    JTextArea creditsJTA = new JTextArea(
      "Copyright (C) 2000, 2012 Kurt Motekew\n" +
      "GNU GPL v2 and\n" +
      "GNU LGPL v2.1\n\n" +
      "\"Earth City Lights\" Globe Image:\n" +
      "\tData courtesy Marc Imhoff of NASA GSFC and\n" +
      "\tChristopher Elvidge of NOAA NGDC.\n" +
      "\tImage by Craig Mayhew and\n" +
      "\tRobert Simmon, NASA GSFC.\n" +
      "http://visibleearth.nasa.gov/"
    );
    creditsJTA.setEditable(false);
    add(creditsJTA);
  }
}
