#include <stdlib.h>
#include <string.h>
#include <locale.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "link.h"
#include "serial.h"
#include "nksdl.h"

#undef main

SDL_RWops *TTF_RW_droid_sans_normal();

enum {
	POPUP_NONE			= 0,
	POPUP_RESET_DEFAULT,
	POPUP_FLASH_ERRNO,
	POPUP_FLASH_CLEANUP,
	POPUP_SYSTEM_REBOOT,
};

struct public {

	struct nk_sdl		*nk;
	struct link_pmc		*lp;

	int			fe_def_size_x;
	int			fe_def_size_y;
	int			fe_font_h;
	int			fe_padding;
	int			fe_base;

	char			combo_fuzzy[80];
	int			combo_count;

	struct {

		int			page_current;
		int			page_selected;
		int			page_pushed;

		void			(* pagetab [32]) (struct public *);
	}
	menu;

	struct {

		int			started;

		struct serial_list	port_list;
		int			port_select;
		int			rate_select;
	}
	serial;

	const char		*popup_msg;
	int			popup_enum;

	char			ln[240];
};

static void
ng_name_label(struct public *pub, const char *name, struct link_reg *reg)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	bounds = nk_widget_bounds(ctx);
	nk_label(ctx, name, NK_TEXT_LEFT);

	if (nk_contextual_begin(ctx, 0, nk_vec2(pub->fe_base * 20, 300), bounds)) {

		nk_layout_row_dynamic(ctx, 0, 1);

		sprintf(pub->ln, "Fetch \"%.77s\"", reg->sym);

		if (nk_contextual_item_label(ctx, pub->ln, NK_TEXT_LEFT)) {

			if (reg->update == 0)
				reg->fetched = 0;
		}

		if (reg->mode & LINK_REG_CONFIG) {

			/* TODO */
		}
		else {
			sprintf(pub->ln, "Update continuously [%s]",
					(reg->update != 0) ? "ON" : "OFF");

			if (nk_contextual_item_label(ctx, pub->ln, NK_TEXT_LEFT)) {

				reg->update = (reg->update == 0) ? 100 : 0;
			}

			if (nk_contextual_item_label(ctx, "Add to telemetry",
						NK_TEXT_LEFT)) {

				/* TODO */
			}
		}

		nk_contextual_end(ctx);
	}
}

static void
ng_name_label_hidden(struct public *pub, const char *name, const char *sym)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	bounds = nk_widget_bounds(ctx);
	nk_label(ctx, name, NK_TEXT_LEFT);

	if (nk_contextual_begin(ctx, 0, nk_vec2(pub->fe_base * 20, 300), bounds)) {

		nk_layout_row_dynamic(ctx, 0, 1);

		sprintf(pub->ln, "Unknown \"%.77s\"", sym, 111);

		nk_contextual_item_label(ctx, pub->ln, NK_TEXT_LEFT);
		nk_contextual_end(ctx);
	}
}

static int
ng_combo_linked(struct public *pub, struct link_reg *reg,
		int item_height, struct nk_vec2 size)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct nk_panel			*layout = ctx->current->layout;

	struct nk_vec2			spacing;
	struct nk_vec2			padding;

	int				max_height;
	int				reg_ID, select_ID;

	spacing = ctx->style.window.spacing;
	padding = ctx->style.window.group_padding;
	max_height = pub->combo_count * (item_height + (int) spacing.y);
	max_height += layout->row.min_height + (int) spacing.y;
	max_height += (int) spacing.y * 2 + (int) padding.y * 2;
	size.y = NK_MIN(size.y, (float) max_height);

	select_ID = reg->lval;

	select_ID = (select_ID >= LINK_REGS_MAX) ? 0
		: (select_ID < 0) ? 0 : select_ID;

	if (nk_combo_begin_label(ctx, lp->reg[select_ID].sym, size)) {

		struct nk_style_button		button;

		button = ctx->style.button;
		button.normal = button.active;
		button.hover = button.active;

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_end(ctx);

		nk_button_symbol_styled(ctx, &button, NK_SYMBOL_TRIANGLE_RIGHT);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD,
				pub->combo_fuzzy, 79, nk_filter_default);

		pub->combo_count = 0;

		nk_layout_row_dynamic(ctx, (float) item_height, 1);

		for (reg_ID = 0; reg_ID < LINK_REGS_MAX; ++reg_ID) {

			if (lp->reg[reg_ID].sym[0] == 0)
				break;

			if (strstr(lp->reg[reg_ID].sym, pub->combo_fuzzy) != NULL) {

				if (nk_combo_item_label(ctx, lp->reg[reg_ID].sym,
							NK_TEXT_LEFT)) {

					select_ID = reg_ID;
				}

				pub->combo_count++;
			}
		}

		if (pub->combo_count == 0)
			pub->combo_count = 1;

		nk_combo_end(ctx);
	}

	return select_ID;
}

static struct nk_rect
ng_get_popup_bounds(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			win, popup;

	win = nk_window_get_content_region(ctx);

	popup.w = (int) (win.w * 0.7f);
	popup.h = (int) (win.h * 0.4f);

	popup.x = win.w / 2 - popup.w / 2;
	popup.y = (int) (win.h * 0.6f);

	return popup;
}

static int
ng_popup_window(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	int				rc = 0;

	if (pub->popup_enum == popup) {

		struct nk_rect		bounds = ng_get_popup_bounds(pub);

		if (nk_popup_begin(ctx, NK_POPUP_DYNAMIC, " ",
					NK_WINDOW_CLOSABLE, bounds)) {

			nk_layout_row_dynamic(ctx, pub->fe_font_h * 4, 1);
			nk_label_wrap(ctx, title);

			nk_layout_row_template_begin(ctx, 0);
			nk_layout_row_template_push_static(ctx, pub->fe_base);
			nk_layout_row_template_push_variable(ctx, 1);
			nk_layout_row_template_push_static(ctx, pub->fe_base);
			nk_layout_row_template_push_variable(ctx, 1);
			nk_layout_row_template_push_static(ctx, pub->fe_base);
			nk_layout_row_template_end(ctx);

			nk_spacer(ctx);

			if (nk_button_label(ctx, "OK")) {

				rc = 1;

				nk_popup_close(ctx);

				pub->popup_enum = 0;
			}

			nk_spacer(ctx);

			if (nk_button_label(ctx, "Cancel")) {

				nk_popup_close(ctx);

				pub->popup_enum = 0;
			}

			nk_spacer(ctx);

			nk_popup_end(ctx);
		}
		else {
			pub->popup_enum = 0;
		}
	}

	return rc;
}

static void
ng_popup_window_info(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	if (pub->popup_enum == popup) {

		struct nk_rect		bounds = ng_get_popup_bounds(pub);

		if (nk_popup_begin(ctx, NK_POPUP_DYNAMIC, " ",
					NK_WINDOW_CLOSABLE, bounds)) {

			nk_layout_row_dynamic(ctx, pub->fe_font_h * 4, 1);
			nk_label_wrap(ctx, title);

			nk_layout_row_template_begin(ctx, 0);
			nk_layout_row_template_push_static(ctx, pub->fe_base);
			nk_layout_row_template_push_variable(ctx, 1);
			nk_layout_row_template_push_static(ctx, pub->fe_base);
			nk_layout_row_template_end(ctx);

			nk_spacer(ctx);

			if (nk_button_label(ctx, "OK")) {

				nk_popup_close(ctx);

				pub->popup_enum = 0;
			}

			nk_spacer(ctx);

			nk_popup_end(ctx);
		}
		else {
			pub->popup_enum = 0;
		}
	}
}

static void
reg_enum_toggle(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_selectable	select;
	int				rc;

	select = ctx->style.selectable;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		ng_name_label(pub, name, reg);

		rc = nk_select_label(ctx, reg->um, NK_TEXT_LEFT, reg->lval);

		if (rc != reg->lval) {

			sprintf(reg->val, "%i", (rc != 0) ? 1 : 0);

			reg->modified = lp->clock;
			reg->lval = rc;
		}

		nk_spacer(ctx);
		nk_label(ctx, reg->val, NK_TEXT_LEFT);
		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		struct nk_color			hidden;

		hidden = nk->table[NK_COLOR_HIDDEN];

		ctx->style.selectable.normal  = nk_style_item_color(hidden);
		ctx->style.selectable.hover   = nk_style_item_color(hidden);
		ctx->style.selectable.pressed = nk_style_item_color(hidden);
		ctx->style.selectable.normal_active  = nk_style_item_color(hidden);
		ctx->style.selectable.hover_active   = nk_style_item_color(hidden);
		ctx->style.selectable.pressed_active = nk_style_item_color(hidden);

		ng_name_label_hidden(pub, name, sym);
		nk_select_label(ctx, " ", NK_TEXT_LEFT, 0);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	ctx->style.selectable = select;
}

static void
reg_enum_errno(struct public *pub, const char *sym, const char *name, int onalert)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_selectable	select;
	struct nk_color			hidden;

	select = ctx->style.selectable;

	hidden = nk->table[NK_COLOR_WINDOW];

	ctx->style.selectable.normal  = nk_style_item_color(hidden);
	ctx->style.selectable.hover   = nk_style_item_color(hidden);
	ctx->style.selectable.pressed = nk_style_item_color(hidden);
	ctx->style.selectable.normal_active  = nk_style_item_color(hidden);
	ctx->style.selectable.hover_active   = nk_style_item_color(hidden);
	ctx->style.selectable.pressed_active = nk_style_item_color(hidden);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.selectable.normal  = nk_style_item_color(flash);
			ctx->style.selectable.hover   = nk_style_item_color(flash);
			ctx->style.selectable.pressed = nk_style_item_color(flash);
			ctx->style.selectable.normal_active  = nk_style_item_color(flash);
			ctx->style.selectable.hover_active   = nk_style_item_color(flash);
			ctx->style.selectable.pressed_active = nk_style_item_color(flash);
		}

		if (		onalert != 0
				&& reg->lval != 0) {

			struct nk_color		alert;

			alert = nk->table[NK_COLOR_FLICKER_ALERT];

			ctx->style.selectable.normal  = nk_style_item_color(alert);
			ctx->style.selectable.hover   = nk_style_item_color(alert);
			ctx->style.selectable.pressed = nk_style_item_color(alert);
			ctx->style.selectable.normal_active  = nk_style_item_color(alert);
			ctx->style.selectable.hover_active   = nk_style_item_color(alert);
			ctx->style.selectable.pressed_active = nk_style_item_color(alert);
		}

		ng_name_label(pub, name, reg);
		nk_select_label(ctx, reg->um, NK_TEXT_LEFT, 0);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label(ctx, reg->val, NK_TEXT_LEFT);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ctx->style.selectable.normal  = nk_style_item_color(hidden);
		ctx->style.selectable.hover   = nk_style_item_color(hidden);
		ctx->style.selectable.pressed = nk_style_item_color(hidden);
		ctx->style.selectable.normal_active  = nk_style_item_color(hidden);
		ctx->style.selectable.hover_active   = nk_style_item_color(hidden);
		ctx->style.selectable.pressed_active = nk_style_item_color(hidden);

		ng_name_label_hidden(pub, name, sym);
		nk_select_label(ctx, " ", NK_TEXT_LEFT, 0);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
	}

	ctx->style.selectable = select;
}

static void
reg_enum_get_item(void *userdata, int n, const char **item)
{
	struct link_reg			*reg = (struct link_reg *) userdata;

	if (reg->combo[n] != NULL) {

		*item = reg->combo[n];
	}
	else {
		*item = " ";
	}
}

static void
reg_enum_combo(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_combo		combo;
	struct nk_color			hidden;
	int				rc;

	combo = ctx->style.combo;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		ng_name_label(pub, name, reg);

		reg->lval = (reg->lval < reg->lmax_combo + 2
				&& reg->lval >= 0) ? reg->lval : 0;

		rc = nk_combo_callback(ctx, &reg_enum_get_item, reg,
				reg->lval, reg->lmax_combo + 2, pub->fe_font_h
				+ pub->fe_padding, nk_vec2(pub->fe_base * 20, 400));

		if (rc != reg->lval) {

			sprintf(reg->val, "%i", rc);

			reg->modified = lp->clock;
		}

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label(ctx, reg->val, NK_TEXT_LEFT);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
	}

	ctx->style.combo = combo;
}

static void
reg_text_large(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_edit		edit;

	edit = ctx->style.edit;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.edit.normal = nk_style_item_color(flash);
			ctx->style.edit.hover  = nk_style_item_color(flash);
			ctx->style.edit.active = nk_style_item_color(flash);
		}

		ng_name_label(pub, name, reg);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				reg->val, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_spacer(ctx);
	}

	ctx->style.edit = edit;
}

static void
reg_float(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_edit		edit;
	struct nk_color			hidden;

	edit = ctx->style.edit;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.edit.normal = nk_style_item_color(flash);
			ctx->style.edit.hover  = nk_style_item_color(flash);
			ctx->style.edit.active = nk_style_item_color(flash);
		}

		ng_name_label(pub, name, reg);

		if (reg->mode & LINK_REG_READ_ONLY) {

			nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
					reg->val, 79, nk_filter_default);
		}
		else {
			int			rc;

			rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
				| NK_EDIT_SIG_ENTER, reg->val, 79, nk_filter_default);

			if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

				reg->modified = lp->clock;
			}
		}

		nk_spacer(ctx);

		if (reg->um[0] != 0) {

			sprintf(pub->ln, "(%.16s)", reg->um);
			nk_label(ctx, pub->ln, NK_TEXT_LEFT);
		}
		else {
			nk_spacer(ctx);
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	ctx->style.edit = edit;
}

static void
reg_float_auto(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_edit		edit;
	struct nk_color			hidden;
	struct nk_vec2			padding;

	edit = ctx->style.edit;
	padding = ctx->style.window.group_padding;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 5);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 4);
	nk_layout_row_template_push_static(ctx, pub->fe_base - padding.y);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL) {

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.edit.normal = nk_style_item_color(flash);
			ctx->style.edit.hover  = nk_style_item_color(flash);
			ctx->style.edit.active = nk_style_item_color(flash);
		}

		ng_name_label(pub, name, reg);

		if (reg->mode & LINK_REG_READ_ONLY) {

			nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
					reg->val, 79, nk_filter_default);
		}
		else {
			int			rc;

			rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
				| NK_EDIT_SIG_ENTER, reg->val, 79, nk_filter_default);

			if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

				reg->modified = lp->clock;
			}
		}

		nk_spacer(ctx);

		if (reg->um[0] != 0) {

			sprintf(pub->ln, "(%.16s)", reg->um);
			nk_label(ctx, pub->ln, NK_TEXT_LEFT);
		}
		else {
			nk_spacer(ctx);
		}

		if (nk_button_label(ctx, "Auto")) {

			sprintf(reg->val, "0");

			reg->modified = lp->clock;
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	ctx->style.edit = edit;
}

static void
reg_um_get_item(void *userdata, int n, const char **item)
{
	struct link_reg			*reg = (struct link_reg *) userdata;

	*item = reg[n].um;
}

static void
reg_float_um(struct public *pub, const char *sym, const char *name, int defsel)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_edit		edit;
	struct nk_color			hidden;
	int				rc, min, max;

	edit = ctx->style.edit;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	rc = link_reg_lookup_range(lp, sym, &min, &max);

	if (rc != 0 && min < max) {

		if (lp->reg[min + defsel].shown == 0) {

			lp->reg[min].um_sel = defsel;
		}

		reg = &lp->reg[min + lp->reg[min].um_sel];

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.edit.normal = nk_style_item_color(flash);
			ctx->style.edit.hover  = nk_style_item_color(flash);
			ctx->style.edit.active = nk_style_item_color(flash);
		}

		ng_name_label(pub, name, reg);

		if (reg->mode & LINK_REG_READ_ONLY) {

			nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
					reg->val, 79, nk_filter_default);
		}
		else {
			rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
				| NK_EDIT_SIG_ENTER, reg->val, 79, nk_filter_default);

			if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

				reg->modified = lp->clock;
			}
		}

		nk_spacer(ctx);

		rc = nk_combo_callback(ctx, &reg_um_get_item, &lp->reg[min],
				lp->reg[min].um_sel, max - min + 1, pub->fe_font_h
				+ pub->fe_padding, nk_vec2(pub->fe_base * 8, 400));

		if (rc != lp->reg[min].um_sel) {

			lp->reg[min].um_sel = rc;
			lp->reg[min + lp->reg[min].um_sel].fetched = 0;
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	ctx->style.edit = edit;
}

static void
reg_linked(struct public *pub, const char *sym, const char *name)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_combo		combo;
	struct nk_color			hidden;
	int				rc;

	combo = ctx->style.combo;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	reg = link_reg_lookup(lp, sym);

	if (reg != NULL && (reg->mode & LINK_REG_LINKED) != 0) {

		ng_name_label(pub, name, reg);

		rc = ng_combo_linked(pub, reg, pub->fe_font_h + pub->fe_padding,
				nk_vec2(pub->fe_base * 20, 400));

		if (rc != reg->lval) {

			sprintf(reg->val, "%i", rc);

			reg->modified = lp->clock;
		}

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label(ctx, reg->val, NK_TEXT_LEFT);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		ng_name_label_hidden(pub, name, sym);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
	}

	ctx->style.combo = combo;
}

static void
reg_float_prog(struct public *pub, int reg_ID, int onactive)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg = NULL;

	struct nk_style_edit		edit;
	struct nk_color			hidden;

	edit = ctx->style.edit;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	if (reg_ID > 1 && reg_ID < LINK_REGS_MAX) {

		reg = &lp->reg[reg_ID];
	}

	if (		reg != NULL
			&& (reg->mode & LINK_REG_CONFIG) == 0) {

		if (reg->fetched + 500 > lp->clock) {

			struct nk_color		flash;

			flash = nk->table[NK_COLOR_FLICKER_LIGHT];

			ctx->style.edit.normal = nk_style_item_color(flash);
			ctx->style.edit.hover  = nk_style_item_color(flash);
			ctx->style.edit.active = nk_style_item_color(flash);
		}

		if (		onactive != 0
				&& reg->fetched + 100 < reg->shown) {

			reg->fetched = 0;
		}

		nk_prog(ctx, 1, 10, nk_false);

		nk_spacer(ctx);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				reg->val, 79, nk_filter_default);

		nk_spacer(ctx);

		if (reg->um[0] != 0) {

			sprintf(pub->ln, "(%.16s)", reg->um);
			nk_label(ctx, pub->ln, NK_TEXT_LEFT);
		}
		else {
			nk_spacer(ctx);
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		nk_prog(ctx, 0, 10, nk_false);
		nk_spacer(ctx);

		pub->ln[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_READ_ONLY,
				pub->ln, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	nk_layout_row_dynamic(ctx, pub->fe_base, 1);
	nk_spacer(ctx);

	ctx->style.edit = edit;
}

static void
page_diagnose(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_style_button		orange;

	const char			*ls_baudrates[] = {

		"1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"
	};

	orange = ctx->style.button;
	orange.normal = nk_style_item_color(nk->table[NK_COLOR_ORANGE_BUTTON]);
	orange.hover = nk_style_item_color(nk->table[NK_COLOR_ORANGE_BUTTON_HOVER]);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 13);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (pub->lp->linked == 0) {

		if (pub->serial.started == 0) {

			serial_enumerate(&pub->serial.port_list);

			pub->serial.started = 1;
			pub->serial.rate_select = 6;
		}

		nk_label(ctx, "Serial port", NK_TEXT_LEFT);
		pub->serial.port_select = nk_combo(ctx, pub->serial.port_list.name,
				pub->serial.port_list.dnum, pub->serial.port_select,
				pub->fe_font_h + pub->fe_padding,
				nk_vec2(pub->fe_base * 13, 400));

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Scan")) {

			serial_enumerate(&pub->serial.port_list);
		}

		nk_spacer(ctx);

		nk_label(ctx, "Baudrate", NK_TEXT_LEFT);
		pub->serial.rate_select = nk_combo(ctx, ls_baudrates, 8,
				pub->serial.rate_select, pub->fe_font_h
				+ pub->fe_padding, nk_vec2(pub->fe_base * 13, 400));

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Connect")) {

			const char		*portname;
			int			baudrate = 0;

			portname = pub->serial.port_list.name[pub->serial.port_select];
			lk_stoi(&baudrate, ls_baudrates[pub->serial.rate_select]);

			link_open(pub->lp, portname, baudrate);
		}
	}
	else {
		nk_label(ctx, "Serial port", NK_TEXT_LEFT);
		nk_label(ctx, pub->lp->devname, NK_TEXT_LEFT);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_spacer(ctx);

		nk_label(ctx, "Baudrate", NK_TEXT_LEFT);
		nk_label(ctx, pub->lp->baudrate, NK_TEXT_LEFT);

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Drop")) {

			link_close(pub->lp);
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Self Test")) {

		link_command(pub->lp, "pm_self_test");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Self Adjust")) {

		link_command(pub->lp, "pm_self_adjust");
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_errno(pub, "pm.fsm_errno", "FSM errno", 1);
	reg_float(pub, "pm.const_fb_U", "DC link voltage");
	reg_float(pub, "pm.ad_IA_0", "A sensor drift");
	reg_float(pub, "pm.ad_IB_0", "B sensor drift");
	reg_text_large(pub, "pm.self_BST", "Bootstrap retention time");
	reg_text_large(pub, "pm.self_BM", "Self test result");
	reg_text_large(pub, "pm.self_RMSi", "Current sensor RMS");
	reg_text_large(pub, "pm.self_RMSu", "Voltage sensor RMS");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.ad_IA_1", "A sensor scale");
	reg_float(pub, "pm.ad_IB_1", "B sensor scale");
	reg_float(pub, "pm.ad_UA_0", "A voltage offset");
	reg_float(pub, "pm.ad_UA_1", "A voltage scale");
	reg_float(pub, "pm.ad_UB_0", "B voltage offset");
	reg_float(pub, "pm.ad_UB_1", "B voltage scale");
	reg_float(pub, "pm.ad_UC_0", "C voltage offset");
	reg_float(pub, "pm.ad_UC_1", "C voltage scale");
	reg_float(pub, "pm.tvm_FIR_A_tau", "A voltage FIR tau");
	reg_float(pub, "pm.tvm_FIR_B_tau", "B voltage FIR tau");
	reg_float(pub, "pm.tvm_FIR_C_tau", "C voltage FIR tau");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.fb_iA", "A current feedback");
	reg_float(pub, "pm.fb_iB", "B current feedback");
	reg_float(pub, "pm.fb_uA", "A voltage feedback");
	reg_float(pub, "pm.fb_uB", "B voltage feedback");
	reg_float(pub, "pm.fb_uC", "A voltage feedback");
	reg_float(pub, "pm.fb_HS", "HALL sensors feedback");
	reg_float(pub, "pm.fb_EP", "ABI encoder feedback");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "PWM 0%")) {

		link_command(pub->lp,	"hal_PWM_set_Z 0" "\r\n"
					"hal_PWM_set_DC 0");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "PWM 50%")) {

		struct link_reg		*reg;
		int			dc_resolution = 2800;

		reg = link_reg_lookup(pub->lp, "pm.dc_resolution");
		if (reg != NULL) { dc_resolution = reg->lval; }

		sprintf(pub->ln,	"hal_PWM_set_Z 0" "\r\n"
					"hal_PWM_set_DC %i", dc_resolution / 2);

		link_command(pub->lp, pub->ln);
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_probe(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 10);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Probe Base")) {

		link_command(pub->lp, "pm_probe_base");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Probe Spinup")) {

		link_command(pub->lp, "pm_probe_spinup");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Probe Detached")) {

		link_command(pub->lp, "pm_probe_detached");
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_errno(pub, "pm.fsm_errno", "FSM errno", 1);
	reg_float(pub, "pm.const_fb_U", "DC link voltage");
	reg_float_um(pub, "pm.const_E", "Motor Kv constant", 1);
	reg_float(pub, "pm.const_R", "Motor winding resistance");
	reg_float(pub, "pm.const_Zp", "Rotor pole pairs number");
	reg_float_um(pub, "pm.const_Ja", "Moment of inertia", 1);
	reg_float(pub, "pm.const_im_L1", "Inductance D");
	reg_float(pub, "pm.const_im_L2", "Inductance Q");
	reg_float(pub, "pm.const_im_B", "Principal angle");
	reg_float(pub, "pm.const_im_R", "Active impedance");
	reg_float(pub, "pm.const_ld_S", "Circumference length");

	reg = link_reg_lookup(pub->lp, "pm.const_Zp");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.const_E");
		if (reg != NULL) { reg += reg->um_sel; reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "FSM Startup")) {

		if (link_command(pub->lp, "pm_fsm_startup") != 0) {

			reg = link_reg_lookup(pub->lp, "pm.lu_mode");
			if (reg != NULL) { reg->fetched = 0; }
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "FSM Shutdown")) {

		if (link_command(pub->lp, "pm_fsm_shutdown") != 0) {

			reg = link_reg_lookup(pub->lp, "pm.lu_mode");
			if (reg != NULL) { reg->fetched = 0; }
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Probe E")) {

		reg = link_reg_lookup(pub->lp, "pm.lu_mode");

		if (reg != NULL && reg->lval != 0) {

			link_command(pub->lp, "pm_probe_const_E");
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Probe J")) {

		reg = link_reg_lookup(pub->lp, "pm.lu_mode");

		if (reg != NULL && reg->lval != 0) {

			link_command(pub->lp, "pm_probe_const_J");
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_mode");

	if (reg != NULL) {

		int		rate;

		if (reg->lval != 0) {

			reg->update = 1000;

			rate = 100;
		}
		else {
			reg->update = 0;

			rate = 0;
		}

		reg = link_reg_lookup(pub->lp, "pm.lu_iD");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_iQ");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_wS");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_wS_rpm");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_wS_mmps");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_wS_kmh");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.lu_lpf_torque");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_enum_errno(pub, "pm.lu_mode", "LU operation mode", 0);
	reg_float(pub, "pm.lu_iD", "LU current D");
	reg_float(pub, "pm.lu_iQ", "LU current Q");
	reg_float_um(pub, "pm.lu_wS", "LU speed estimate", 1);
	reg_float(pub, "pm.lu_lpf_torque", "LU torque estimate");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_um(pub, "pm.i_setpoint_torque", "Torque setpoint", 0);
	reg_float_um(pub, "pm.s_setpoint_speed", "Speed setpoint", 1);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_HAL(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_float(pub, "hal.USART_baud_rate", "USART baudrate");
	reg_float(pub, "hal.PWM_frequency", "PWM frequency");
	reg_float(pub, "hal.PWM_deadtime", "PWM deadtime");
	reg_float(pub, "hal.ADC_reference_voltage", "ADC reference voltage");
	reg_float(pub, "hal.ADC_shunt_resistance", "Current shunt resistance");
	reg_float(pub, "hal.ADC_amplifier_gain", "Current amplifier gain");
	reg_float(pub, "hal.ADC_voltage_ratio", "DC link voltage divider ratio");
	reg_float(pub, "hal.ADC_terminal_ratio", "Terminal voltage divider ratio");
	reg_float(pub, "hal.ADC_terminal_bias", "Terminal voltage bias");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "hal.TIM_mode", "TIM operation mode");
	reg_enum_combo(pub, "hal.PPM_mode", "PPM operation mode");
	reg_float(pub, "hal.PPM_timebase", "PPM time base");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_in_network(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
}

static void
page_in_STEPDIR(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
}

static void
page_in_PPM(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "hal.PPM_signal_caught");

	if (reg != NULL) {

		int		rate;

		reg->update = 200;
		reg->shown = pub->lp->clock;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "hal.PPM_get_PERIOD");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "hal.PPM_get_PULSE");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_float(pub, "hal.PPM_signal_caught", "PPM signal caught");
	reg_float(pub, "hal.PPM_get_PERIOD", "PPM period received");
	reg_float(pub, "hal.PPM_get_PULSE", "PPM pulse received");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_linked(pub, "ap.ppm_reg_ID", "Control register ID");
	reg_enum_toggle(pub, "ap.ppm_STARTUP", "Startup on signal caught");
	reg_float(pub, "ap.ppm_in_range_0", "Input range 0");
	reg_float(pub, "ap.ppm_in_range_1", "Input range 1");
	reg_float(pub, "ap.ppm_in_range_2", "Input range 2");
	reg_float(pub, "ap.ppm_control_range_0", "Control range 0");
	reg_float(pub, "ap.ppm_control_range_1", "Control range 1");
	reg_float(pub, "ap.ppm_control_range_2", "Control range 2");

	reg = link_reg_lookup(pub->lp, "ap.ppm_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range_0");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range_1");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range_2");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_in_analog(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "hal.ADC_get_analog_ANG");
	if (reg != NULL) { reg->update = 100; }

	reg = link_reg_lookup(pub->lp, "hal.ADC_get_analog_BRK");
	if (reg != NULL) { reg->update = 100; }

	reg_float(pub, "hal.ADC_get_analog_ANG", "ANG voltage");
	reg_float(pub, "hal.ADC_get_analog_BRK", "BRK voltage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "ap.analog_ENABLED", "Analog mode");
	reg_linked(pub, "ap.analog_reg_ID", "Control register ID");
	reg_enum_toggle(pub, "ap.analog_STARTUP", "Startup if ANG is in range");
	reg_float(pub, "ap.analog_in_ANG_0", "ANG range 0");
	reg_float(pub, "ap.analog_in_ANG_1", "ANG range 1");
	reg_float(pub, "ap.analog_in_ANG_2", "ANG range 2");
	reg_float(pub, "ap.analog_in_BRK_0", "BRK range 0");
	reg_float(pub, "ap.analog_in_BRK_1", "BRK range 1");
	reg_float(pub, "ap.analog_in_lost_0", "Lost range 0");
	reg_float(pub, "ap.analog_in_lost_1", "Lost range 1");
	reg_float(pub, "ap.analog_control_ANG_0", "Control range 0");
	reg_float(pub, "ap.analog_control_ANG_1", "Control range 1");
	reg_float(pub, "ap.analog_control_ANG_2", "Control range 2");
	reg_float(pub, "ap.analog_control_BRK", "BRK control");

	reg = link_reg_lookup(pub->lp, "ap.analog_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.analog_control_ANG_0");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.analog_control_ANG_1");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.analog_control_ANG_2");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.analog_control_BRK");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "ap.idle_ENABLED", "IDLE mode");
	reg_linked(pub, "ap.idle_reg_ID", "IDLE register ID");
	reg_float(pub, "ap.idle_control_tol_0", "IDLE range 0");
	reg_float(pub, "ap.idle_control_tol_1", "IDLE range 1");
	reg_float(pub, "ap.idle_TIME_s", "IDLE timeout");

	reg = link_reg_lookup(pub->lp, "ap.idle_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.idle_control_tol_0");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "ap.idle_control_tol_1");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_thermal(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "ap.ntc_PCB.r_balance", "NTC balance (PCB)");
	reg_float(pub, "ap.ntc_PCB.r_ntc_0", "NTC resistance at Ta");
	reg_float(pub, "ap.ntc_PCB.ta_0", "NTC Ta (PCB)");
	reg_float(pub, "ap.ntc_PCB.betta", "NTC betta (PCB)");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.ntc_EXT.r_balance", "NTC balance (EXT)");
	reg_float(pub, "ap.ntc_EXT.r_ntc_0", "NTC resistance at Ta");
	reg_float(pub, "ap.ntc_EXT.ta_0", "NTC Ta (EXT)");
	reg_float(pub, "ap.ntc_EXT.betta", "NTC betta (EXT)");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "ap.temp_PCB");
	if (reg != NULL) { reg->update = 1000; }

	reg = link_reg_lookup(pub->lp, "ap.temp_EXT");
	if (reg != NULL) { reg->update = 1000; }

	reg = link_reg_lookup(pub->lp, "ap.temp_INT");
	if (reg != NULL) { reg->update = 1000; }

	reg_float(pub, "ap.temp_PCB", "Temperature PCB");
	reg_float(pub, "ap.temp_EXT", "Temperature EXT");
	reg_float(pub, "ap.temp_INT", "Temperature Core");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.heat_PCB_halt", "Halt threshold (PCB)");
	reg_float(pub, "ap.heat_PCB_on_FAN", "Fan ON threshold (PCB)");
	reg_float(pub, "ap.heat_PCB_derated", "Derated current (PCB)");
	reg_float(pub, "ap.heat_EXT_halt", "Halt threshold (EXT)");
	reg_float(pub, "ap.heat_EXT_derated", "Derated current (EXT)");
	reg_float(pub, "ap.heat_recovery_gap", "Halt recovery gap");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_config(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	reg_float(pub, "pm.dc_resolution", "PWM resolution");
	reg_float(pub, "pm.dc_minimal", "Minimal pulse");
	reg_float(pub, "pm.dc_clearance", "Clearance before ADC sample");
	reg_float(pub, "pm.dc_skip", "Skip after ADC sample");
	reg_float(pub, "pm.dc_bootstrap", "Bootstrap retention time");
	reg_float(pub, "pm.dc_clamped", "Bootstrap recharge time");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "pm.config_NOP", "Number of motor phases");
	reg_enum_toggle(pub, "pm.config_TVM", "Terminal voltage measurement");
	reg_enum_combo(pub, "pm.config_IFB", "Current measurement method");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.config_VSI_CIRCULAR", "Circular SVPWM clamping");
	reg_enum_toggle(pub, "pm.config_VSI_PRECISE", "Precise SVPWM in middle");
	reg_enum_toggle(pub, "pm.config_LU_FORCED", "Forced control");
	reg_enum_toggle(pub, "pm.config_LU_ESTIMATE_FLUX", "Sensorless FLUX");
	reg_enum_toggle(pub, "pm.config_LU_ESTIMATE_HFI", "Sensorless HFI");
	reg_enum_toggle(pub, "pm.config_LU_SENSOR_HALL", "Discrete HALL sensors");
	reg_enum_toggle(pub, "pm.config_LU_SENSOR_ABI", "ABI incremental encoder");
	reg_enum_combo(pub, "pm.config_FA_PRECEDENCE", "Servo precedence");
	reg_enum_combo(pub, "pm.config_DRIVE", "Drive loop");
	reg_enum_toggle(pub, "pm.config_HFI_MAJOR", "HFI major axis inverse");
	reg_enum_toggle(pub, "pm.config_HOLDING_BRAKE", "Holding brake feature");
	reg_enum_toggle(pub, "pm.config_SPEED_LIMITED", "Speed limit feature");
	reg_enum_toggle(pub, "pm.config_MTPA_RELUCTANCE", "MTPA control");
	reg_enum_toggle(pub, "pm.config_WEAKENING", "Flux weakening");
	reg_enum_toggle(pub, "pm.config_MILEAGE_INFO", "Mileage info");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.tm_transient_slow", "Transient time (slow)");
	reg_float(pub, "pm.tm_transient_fast", "Transient time (fast)");
	reg_float(pub, "pm.tm_voltage_hold", "Voltage hold time");
	reg_float(pub, "pm.tm_current_hold", "Current hold time");
	reg_float(pub, "pm.tm_instant_probe", "Instant probe time");
	reg_float(pub, "pm.tm_average_probe", "Average probe time");
	reg_float(pub, "pm.tm_average_drift", "Average drift time");
	reg_float(pub, "pm.tm_average_inertia", "Average inertia time");
	reg_float(pub, "pm.tm_startup", "Startup time");
	reg_float(pub, "pm.tm_halt_pause", "Halt pause");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.probe_current_hold", "Probe hold current");
	reg_float(pub, "pm.probe_hold_angle", "Probe hold angle");
	reg_float(pub, "pm.probe_current_weak", "Probe weak current");
	reg_float(pub, "pm.probe_current_sine", "Probe sine current");
	reg_float(pub, "pm.probe_freq_sine_hz", "Probe sine frequency");
	reg_float(pub, "pm.probe_speed_maximal_pc", "Probe speed percentage");
	reg_float_um(pub, "pm.probe_speed_hold", "Probe speed", 0);
	reg_float_um(pub, "pm.probe_speed_detached", "Probe speed (detached)", 0);
	reg_float(pub, "pm.probe_gain_P", "Probe loop gain P");
	reg_float(pub, "pm.probe_gain_I", "Probe loop gain I");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.fault_voltage_tol", "Voltage tolerance");
	reg_float(pub, "pm.fault_current_tol", "Current tolerance");
	reg_float(pub, "pm.fault_accuracy_tol", "Accuracy tolerance");
	reg_float_auto(pub, "pm.fault_current_halt", "Current halt threshold");
	reg_float(pub, "pm.fault_voltage_halt", "Voltage halt threshold");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.lu_transition", "LU transition rate");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Default")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_RESET_DEFAULT;
		}
	}

	if (ng_popup_window(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset PMC configuration") != 0) {

		link_command(pub->lp, "pm_default");
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lu_forced(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_float(pub, "pm.forced_hold_D", "Forced hold current");
	reg_float_um(pub, "pm.forced_maximal", "Maximal forward speed", 0);
	reg_float_um(pub, "pm.forced_reverse", "Maximal reverse speed", 0);
	reg_float_um(pub, "pm.forced_accel", "Forced acceleration", 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Auto")) {

		struct link_reg		*reg;

		if (link_command(pub->lp, "reg pm.forced_maximal 0") != 0) {

			reg = link_reg_lookup(pub->lp, "pm.forced_maximal");
			if (reg != NULL) { reg += reg->um_sel; reg->fetched = 0; }

			reg = link_reg_lookup(pub->lp, "pm.forced_reverse");
			if (reg != NULL) { reg += reg->um_sel; reg->fetched = 0; }

			reg = link_reg_lookup(pub->lp, "pm.forced_accel");
			if (reg != NULL) { reg += reg->um_sel; reg->fetched = 0; }
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_lu_FLUX(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "pm.flux_E", "FLUX linkage");
	reg_float(pub, "pm.flux_F_g", "FLUX position estimate");
	reg_float_um(pub, "pm.flux_wS", "FLUX speed estimate", 0);
	reg_enum_errno(pub, "pm.flux_mode", "FLUX mode", 0);
	reg_float_um(pub, "pm.flux_lpf_wS", "FLUX speed LPF", 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.detach_take_U", "Detached take voltage");
	reg_float(pub, "pm.detach_gain_AD", "Detached adaptive gain");
	reg_float(pub, "pm.detach_gain_SF", "Detached speed gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.flux_gain_IN", "FLUX initial gain");
	reg_float(pub, "pm.flux_gain_LO", "FLUX low speed gain");
	reg_float(pub, "pm.flux_gain_HI", "FLUX high speed gain");
	reg_float(pub, "pm.flux_gain_AD", "FLUX adaptive gain");
	reg_float(pub, "pm.flux_gain_SF", "FLUX speed loop gain");
	reg_float(pub, "pm.flux_gain_IF", "FLUX torque gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_auto(pub, "pm.flux_MPPE", "Random peak to peak noise");
	reg_float(pub, "pm.flux_URE", "Uncertainty due to resistance");
	reg_float(pub, "pm.flux_gain_TAKE", "Take threshold");
	reg_float(pub, "pm.flux_gain_GIVE", "Give threshold");

	reg = link_reg_lookup(pub->lp, "pm.flux_MPPE");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.flux_URE");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "pm.flux_gain_TAKE");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "pm.flux_gain_GIVE");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lu_HFI(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
}

static void
page_lu_HALL(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
}

static void
page_lu_ABI(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
}

static void
page_lu_SINCOS(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	/* TODO */
}

static void
page_wattage(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "pm.watt_wP_maximal", "Maximal consumption");
	reg_float(pub, "pm.watt_iDC_maximal", "Maximal battery current");
	reg_float(pub, "pm.watt_wP_reverse", "Maximal regeneration");
	reg_float(pub, "pm.watt_iDC_reverse", "Maximal battery reverse");
	reg_float(pub, "pm.watt_dclink_HI", "DC link voltage high");
	reg_float(pub, "pm.watt_dclink_LO", "DC link voltage low");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_mode");

	if (reg != NULL) {

		int		rate;

		reg->update = 1000;
		reg->shown = pub->lp->clock;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "pm.watt_lpf_wP");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_float(pub, "pm.watt_lpf_wP", "Wattage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.watt_gain_LP_F", "Voltage LPF gain");
	reg_float(pub, "pm.watt_gain_LP_P", "Wattage LPF gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_current(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float_um(pub, "pm.i_setpoint_torque", "Torque setpoint", 0);
	reg_float_auto(pub, "pm.i_maximal", "Maximal forward current");
	reg_float_auto(pub, "pm.i_reverse", "Maximal reverse current");
	reg_float(pub, "pm.i_derated_HFI", "Derated from HFI");
	reg_float(pub, "pm.i_slew_rate", "Slew rate");
	reg_float(pub, "pm.i_tol_Z", "Dead Zone");
	reg_float_auto(pub, "pm.i_gain_P", "Current LOOP gain P");
	reg_float(pub, "pm.i_gain_I", "Current LOOP gain I");

	reg = link_reg_lookup(pub->lp, "pm.i_gain_P");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.i_slew_rate");
		if (reg != NULL) { reg->fetched = 0; }

		reg = link_reg_lookup(pub->lp, "pm.i_gain_I");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.weak_maximal", "Maximal weakening");
	reg_float(pub, "pm.weak_gain_EU", "Weak gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.v_maximal", "Maximal voltage");
	reg_float(pub, "pm.v_reverse", "Maximal reverse");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_speed(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float_um(pub, "pm.s_setpoint_speed", "Speed setpoint", 0);
	reg_float_um(pub, "pm.s_maximal", "Maximal forward speed", 0);
	reg_float_um(pub, "pm.s_reverse", "Maximal reverse speed", 0);
	reg_float_um(pub, "pm.s_accel", "Maximal acceleration", 0);
	reg_float(pub, "pm.s_linspan", "Regulation span");
	reg_float(pub, "pm.s_tol_Z", "Dead Zone");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_auto(pub, "pm.s_gain_P", "Speed loop gain P");
	reg_float(pub, "pm.s_gain_S", "Speed loop gain S");
	reg_float(pub, "pm.lu_gain_TF", "Torque estimate gain");

	reg = link_reg_lookup(pub->lp, "pm.s_gain_P");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.lu_gain_TF");
		if (reg != NULL) { reg->fetched = 0; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_servo(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

}

static void
page_mileage(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_float(pub, "pm.im_total_revol", "Total revolutions");
	reg_float_um(pub, "pm.im_distance", "Total distance traveled", 1);
	reg_float_um(pub, "pm.im_consumed", "Consumed energy", 0);
	reg_float_um(pub, "pm.im_reverted", "Reverted energy", 0);
	reg_float(pub, "pm.im_capacity_Ah", "Battery capacity");
	reg_float(pub, "pm.im_fuel_pc", "Fuel gauge");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_telemetry(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "tlm.freq_grab_hz", "Frequency of single grab");
	reg_float(pub, "tlm.freq_live_hz", "Live data frequency");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_linked(pub, "tlm.reg_ID_0", "Tele register ID");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_0");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0, 0);

	reg_linked(pub, "tlm.reg_ID_1", "Tele register ID");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_1");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0, 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_flash_block_colored(struct nk_sdl *nk, const int sym)
{
	struct nk_context		*ctx = &nk->ctx;

	struct nk_style_selectable	select;
	struct nk_color			colored;

	select = ctx->style.selectable;

	if (sym == 'a') {

		colored = nk->table[NK_COLOR_ENABLED];
	}
	else if (sym == 'x') {

		colored = nk->table[NK_COLOR_FLICKER_LIGHT];
	}
	else {
		colored = nk->table[NK_COLOR_HIDDEN];
	}

	ctx->style.selectable.normal  = nk_style_item_color(colored);
	ctx->style.selectable.hover   = nk_style_item_color(colored);
	ctx->style.selectable.pressed = nk_style_item_color(colored);
	ctx->style.selectable.normal_active  = nk_style_item_color(colored);
	ctx->style.selectable.hover_active   = nk_style_item_color(colored);
	ctx->style.selectable.pressed_active = nk_style_item_color(colored);

	nk_select_label(ctx, " ", NK_TEXT_LEFT, 0);

	ctx->style.selectable = select;
}

static void
page_flash(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	int				page, block, len, sym;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Program")) {

		link_command(pub->lp,	"flash_prog" "\r\n"
					"flash_info_map");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Info")) {

		link_command(pub->lp, "flash_info_map");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Cleanup")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_FLASH_CLEANUP;
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Reboot")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_SYSTEM_REBOOT;
		}
	}

	nk_spacer(ctx);

	if (ng_popup_window(pub, POPUP_FLASH_CLEANUP,
				"Please confirm that you really"
				" want to cleanup flash storage") != 0) {

		link_command(pub->lp,	"flash_cleanup" "\r\n"
					"flash_info_map");
	}

	if (ng_popup_window(pub, POPUP_SYSTEM_REBOOT,
				"Please confirm that you really"
				" want to reboot PMC") != 0) {

		link_command(pub->lp, "rtos_reboot");
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 300);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "info", NK_WINDOW_BORDER | NK_WINDOW_DYNAMIC)) {

		len = strlen(lp->flash_map[0]);

		nk_layout_row_template_begin(ctx, 0);

		for (block = 0; block < len; ++block) {

			nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
		}

		nk_layout_row_template_end(ctx);

		for (page = 0; page < 8; ++page) {

			if (strlen(lp->flash_map[page]) == 0)
				break;

			for (block = 0; block < len; ++block) {

				sym = lp->flash_map[page][block];

				page_flash_block_colored(nk, sym);
			}
		}

		nk_group_end(ctx);
	}

	if (lp->flash_errno > 0) {

		if (lp->flash_errno == 1) {

			pub->popup_msg = "Flash programming was successful";
		}
		else if (lp->flash_errno == 2) {

			pub->popup_msg = "Flash programming failed";
		}
		else if (lp->flash_errno == 3) {

			pub->popup_msg = "Unable to flash when PM is running";
		}

		lp->flash_errno = 0;
		pub->popup_enum = POPUP_FLASH_ERRNO;
	}

	ng_popup_window_info(pub, POPUP_FLASH_ERRNO, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
menu_select_button(struct public *pub, const char *title, void (* pfunc) (struct public *))
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_style_button		button;

	button = ctx->style.button;
	button.text_alignment = NK_TEXT_LEFT;

	if (pub->menu.page_current == pub->menu.page_selected) {

		button.normal = button.active;
		button.hover = button.active;
		button.text_normal = button.text_active;
		button.text_hover = button.text_active;
	}

	if (nk_button_label_styled(ctx, &button, title)) {

		if (pub->menu.page_current != pub->menu.page_selected) {

			nk_group_set_scroll(ctx, "page", 0, 0);

			pub->menu.page_pushed = pub->menu.page_current;
		}
	}

	pub->menu.pagetab[pub->menu.page_current++] = pfunc;
}

static void
menu_group_layout(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			window;
	struct nk_vec2			padding;

	window = nk_window_get_content_region(ctx);
	padding = ctx->style.window.padding;

	nk_layout_row_template_begin(ctx, window.h - padding.y * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_end(ctx);

	if (nk_group_begin(ctx, "menu", 0)) {

		pub->menu.page_current = 0;

		nk_layout_row_dynamic(ctx, 0, 1);

		menu_select_button(pub, "Diagnose", &page_diagnose);
		menu_select_button(pub, "Probe", &page_probe);
		menu_select_button(pub, "HAL", &page_HAL);
		menu_select_button(pub, "in Network", &page_in_network);
		menu_select_button(pub, "in STEP/DIR", &page_in_STEPDIR);
		menu_select_button(pub, "in PPM", &page_in_PPM);
		menu_select_button(pub, "in Analog", &page_in_analog);
		menu_select_button(pub, "Thermal", &page_thermal);
		menu_select_button(pub, "Config", &page_config);
		menu_select_button(pub, "lu Forced", &page_lu_forced);
		menu_select_button(pub, "lu FLUX", &page_lu_FLUX);
		menu_select_button(pub, "lu HFI", &page_lu_HFI);
		menu_select_button(pub, "lu HALL", &page_lu_HALL);
		menu_select_button(pub, "lu ABI", &page_lu_ABI);
		menu_select_button(pub, "Wattage", &page_wattage);
		menu_select_button(pub, "lp Current", &page_current);
		menu_select_button(pub, "lp Speed", &page_speed);
		menu_select_button(pub, "lp Servo", &page_servo);
		menu_select_button(pub, "Mileage", &page_mileage);
		menu_select_button(pub, "Telemetry", &page_telemetry);
		menu_select_button(pub, "Flash", &page_flash);

		pub->menu.page_selected = pub->menu.page_pushed;

		nk_group_end(ctx);
	}

	if (nk_group_begin(ctx, "page", 0)) {

		(void) pub->menu.pagetab[pub->menu.page_selected] (pub);

		nk_group_end(ctx);
	}
}

int main(int argc, char **argv)
{
	struct nk_sdl		*nk;
	struct link_pmc		*lp;
	struct public		*pub;

	setlocale(LC_NUMERIC, "C");

	nk = calloc(1, sizeof(struct nk_sdl));
	lp = calloc(1, sizeof(struct link_pmc));
	pub = calloc(1, sizeof(struct public));

	pub->nk = nk;
	pub->lp = lp;

	if (0) {

		pub->fe_def_size_x = 900;
		pub->fe_def_size_y = 600;
		pub->fe_font_h = 18;
		pub->fe_padding = 5;
		pub->fe_base = pub->fe_font_h + 1;
	}
	else {
		pub->fe_def_size_x = 1200;
		pub->fe_def_size_y = 900;
		pub->fe_font_h = 26;
		pub->fe_padding = 10;
		pub->fe_base = pub->fe_font_h - 2;
	}

	strcpy(pub->combo_fuzzy, "setpoint");
	pub->combo_count = 10;

	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
	TTF_Init();

	nk->window = SDL_CreateWindow("PMCFE", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			pub->fe_def_size_x, pub->fe_def_size_y, SDL_WINDOW_RESIZABLE);

	SDL_SetWindowMinimumSize(nk->window, pub->fe_def_size_x, pub->fe_def_size_y);
	SDL_StopTextInput();

	nk->fb = SDL_GetWindowSurface(nk->window);
	nk->surface = SDL_CreateRGBSurfaceWithFormat(0, nk->fb->w,
			nk->fb->h, 32, SDL_PIXELFORMAT_XRGB8888);

	nk->ttf_font = TTF_OpenFontRW(TTF_RW_droid_sans_normal(), 1, pub->fe_font_h);

	TTF_SetFontHinting(nk->ttf_font, TTF_HINTING_NORMAL);

	nk->font.userdata.ptr = nk->ttf_font;
        nk->font.height = TTF_FontHeight(nk->ttf_font);
        nk->font.width = &nk_sdl_text_width;

	nk_init_default(&nk->ctx, &nk->font);
	nk_sdl_style_custom(nk, pub->fe_padding);

	SDL_StartTextInput();

	while (nk->onquit == 0) {

		nk->clock = SDL_GetTicks();

		nk_input_begin(&nk->ctx);

		while (SDL_PollEvent(&nk->event) != 0) {

			nk_sdl_input_event(nk, &nk->event);

			nk->active = 1;
		}

		nk_input_end(&nk->ctx);

		if (link_fetch(pub->lp, nk->clock) != 0) {

			nk->active = 1;
		}

		if (nk->active != 0) {

			nk->idled = 0;
		}

		if (nk->updated + 100 < nk->clock) {

			nk->idled += 1;
			nk->active = (nk->idled < 10) ? 1 : 0;
		}

		if (nk->active != 0) {

			struct nk_rect		bounds = nk_rect(0, 0, nk->surface->w,
									nk->surface->h);

			if (lp->hwinfo[0] != 0) {

				nk->ctx.style.window.header.active =
					nk_style_item_color(nk->table[NK_COLOR_ENABLED]);

				sprintf(pub->ln, " %.77s", lp->hwinfo);

				if (lp->network[0] != 0) {

					sprintf(pub->ln + strlen(pub->ln),
							" [%.32s]", lp->network);
				}
			}
			else {
				nk->ctx.style.window.header.active =
					nk_style_item_color(nk->table[NK_COLOR_HEADER]);

				pub->ln[0] = 0;
			}

			if (nk_begin(&nk->ctx, pub->ln, bounds, NK_WINDOW_TITLE)) {

				menu_group_layout(pub);

				nk_end(&nk->ctx);
			}

			nk_sdl_render(nk);

			SDL_BlitSurface(nk->surface, NULL, nk->fb, NULL);
			SDL_UpdateWindowSurface(nk->window);

			nk->updated = nk->clock;
			nk->active = 0;
		}

		link_push(pub->lp);

		SDL_Delay(10);
	}

	link_close(lp);

	free(nk);
	free(lp);
	free(pub);

	SDL_Quit();

	return 0;
}

