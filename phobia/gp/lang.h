/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2023 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_LANG_
#define _H_LANG_

typedef struct {

	const char	*global_menu;
	const char	*global_zoom_menu;
	const char	*global_appearance_menu;
	const char	*global_data_menu;
	const char	*global_lang_menu;
	const char	*dataset_menu[5];

	const char	*axis_menu;
	const char	*axis_zoom_menu;
	const char	*figure_menu;
	const char	*figure_edit_menu;
	const char	*figure_edit_drawing_menu;
	const char	*figure_edit_color_menu;
	const char	*figure_operation_menu;
	const char	*legend_menu;
	const char	*cancel_menu;

	const char	*page_label_edit;
	const char	*figure_label_edit;
	const char	*axis_label_edit;
	const char	*scale_offset_edit;
	const char	*file_name_edit;
	const char	*bit_number_edit;
	const char	*low_pass_edit;
	const char	*polynomial_edit;
	const char	*length_edit;
	const char	*figure_thickness_edit;
	const char	*font_size_edit;
}
lang_t;

enum {
	LANG_EN		= 0,
	LANG_RU,
	LANG_END_OF_LIST
};

void langFill(lang_t *la, int lang);

#endif /* _H_LANG_ */

