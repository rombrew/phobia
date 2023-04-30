#include <stdlib.h>
#include <string.h>
#include <locale.h>
#include <math.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>

#include "gp/gp.h"

#include "config.h"
#include "dirent.h"
#include "link.h"
#include "serial.h"
#include "nksdl.h"

#undef main

#define PHOBIA_FILE_MAX				200
#define PHOBIA_NODE_MAX				32
#define PHOBIA_TAB_MAX				40

SDL_RWops *TTF_RW_droid_sans_normal();

enum {
	POPUP_NONE			= 0,
	POPUP_LINK_PROGRESS,
	POPUP_RESET_DEFAULT,
	POPUP_UNABLE_WARNING,
	POPUP_FLASH_WIPE,
	POPUP_SYSTEM_REBOOT,
	POPUP_TELEMETRY_FLUSH
};

enum {
	DRAWING_EMPTY			= 0,
	DRAWING_WITH_TOOTH,
	DRAWING_WITH_HALL
};

struct public {

	struct config_phobia	*fe;
	struct nk_sdl		*nk;
	struct link_pmc		*lp;

	int			fe_def_size_x;
	int			fe_def_size_y;
	int			fe_font_h;
	int			fe_padding;
	int			fe_base;

	int			fuzzy_count;

	struct {

		int			page_current;
		int			page_selected;
		int			page_pushed;

		void			(* pagetab [PHOBIA_TAB_MAX]) (struct public *);
	}
	menu;

	struct {

		int			started;

		struct serial_list	list;

		int			selected;
		int			baudrate;
	}
	serial;

	struct {

		int			selected;
	}
	network;

	struct {

		struct dirent_stat	sb;

		struct {

			char			name[DIRENT_PATH_MAX];
			char			time[24];
			char			size[24];
		}
		file[PHOBIA_FILE_MAX];

		int			selected;
	}
	scan;

	struct {

		int			wait_GP;

		char			file_snap[PHOBIA_PATH_MAX];
		char			file_grab[PHOBIA_NAME_MAX];
	}
	telemetry;

	gp_t			*gp;

	int			gp_ID;

	struct {

		char			file_snap[PHOBIA_PATH_MAX];
		char			file_grab[PHOBIA_NAME_MAX];
	}
	config;

	char			popup_msg[LINK_MESSAGE_MAX];
	int			popup_enum;

	char			lbuf[PHOBIA_PATH_MAX];
};

static void
pub_font_layout(struct public *pub)
{
	struct config_phobia		*fe = pub->fe;
	struct nk_sdl			*nk = pub->nk;

	if (fe->windowsize == 1) {

		pub->fe_def_size_x = 1200;
		pub->fe_def_size_y = 900;
		pub->fe_font_h = 26;
		pub->fe_padding = 5;
		pub->fe_base = pub->fe_font_h - 2;
	}
	else if (fe->windowsize == 2) {

		pub->fe_def_size_x = 1600;
		pub->fe_def_size_y = 1200;
		pub->fe_font_h = 34;
		pub->fe_padding = 5;
		pub->fe_base = pub->fe_font_h - 2;
	}
	else {
		pub->fe_def_size_x = 900;
		pub->fe_def_size_y = 600;
		pub->fe_font_h = 18;
		pub->fe_padding = 5;
		pub->fe_base = pub->fe_font_h - 0;
	}

	if (nk->ttf_font != NULL) {

		TTF_CloseFont(nk->ttf_font);
	}

	nk->ttf_font = TTF_OpenFontRW(TTF_RW_droid_sans_normal(), 1, pub->fe_font_h);

	TTF_SetFontHinting(nk->ttf_font, TTF_HINTING_NORMAL);

	nk->font.userdata.ptr = nk->ttf_font;
	nk->font.height = TTF_FontHeight(nk->ttf_font);
	nk->font.width = &nk_sdl_text_width;

	SDL_SetWindowMinimumSize(nk->window, pub->fe_def_size_x, pub->fe_def_size_y);
	SDL_SetWindowSize(nk->window, pub->fe_def_size_x, pub->fe_def_size_y);
}

static void
pub_name_label(struct public *pub, const char *name, struct link_reg *reg)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	bounds = nk_widget_bounds(ctx);

	if (reg->mode & LINK_REG_CONFIG) {

		nk_label_colored(ctx, name, NK_TEXT_LEFT, nk->table[NK_COLOR_CONFIG]);
	}
	else {
		nk_label(ctx, name, NK_TEXT_LEFT);
	}

	if (nk_contextual_begin(ctx, 0, nk_vec2(pub->fe_base * 22, 300), bounds)) {

		nk_layout_row_dynamic(ctx, 0, 1);

		sprintf(pub->lbuf, "Fetch \"%.77s\"", reg->sym);

		if (nk_contextual_item_label(ctx, pub->lbuf, NK_TEXT_LEFT)) {

			if (reg->update == 0) {

				reg->onefetch = 1;
			}
		}

		if (reg->mode & LINK_REG_CONFIG) {

			if (		   strcmp(reg->sym, "pm.i_maximal") == 0
					|| strcmp(reg->sym, "pm.probe_speed_hold") == 0
					|| strcmp(reg->sym, "pm.zone_speed_threshold") == 0
					|| strcmp(reg->sym, "pm.forced_maximal") == 0
					|| strcmp(reg->sym, "pm.forced_accel") == 0
					|| strcmp(reg->sym, "pm.i_gain_P") == 0
					|| strcmp(reg->sym, "pm.s_gain_P") == 0
					|| strcmp(reg->sym, "pm.fault_current_halt") == 0) {

				if (nk_contextual_item_label(ctx, "Request configure",
							NK_TEXT_LEFT)) {

					strcpy(reg->val, "0");

					reg->modified = pub->lp->clock;
				}
			}
		}
		else {
			int			rc;

			rc = nk_check_label(ctx, "Continuous update",
					(reg->update != 0) ? nk_true : nk_false);

			reg->update = (rc != 0) ? 100 : 0;
		}

		nk_contextual_end(ctx);
	}
}

static void
pub_name_label_hidden(struct public *pub, const char *name, const char *sym)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	bounds = nk_widget_bounds(ctx);

	nk_label_colored(ctx, name, NK_TEXT_LEFT, nk->table[NK_COLOR_HIDDEN]);

	if (nk_contextual_begin(ctx, 0, nk_vec2(pub->fe_base * 22, 300), bounds)) {

		nk_layout_row_dynamic(ctx, 0, 1);

		sprintf(pub->lbuf, "Unknown \"%.77s\"", sym);

		nk_contextual_item_label(ctx, pub->lbuf, NK_TEXT_LEFT);
		nk_contextual_end(ctx);
	}
}

static int
pub_combo_linked(struct public *pub, struct link_reg *reg,
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
	max_height = pub->fuzzy_count * (item_height + (int) spacing.y);
	max_height += layout->row.min_height + (int) spacing.y;
	max_height += (int) spacing.y * 2 + (int) padding.y * 2;
	size.y = NK_MIN(size.y, (float) max_height);

	select_ID = (reg->lval >= lp->reg_MAX_N) ? 0
		: (reg->lval < 0) ? 0 : reg->lval;

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
				pub->fe->fuzzy, sizeof(pub->fe->fuzzy),
				nk_filter_default);

		pub->fuzzy_count = 0;

		nk_layout_row_dynamic(ctx, (float) item_height, 1);

		for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

			if (lp->reg[reg_ID].sym[0] != 0) {

				if (strstr(lp->reg[reg_ID].sym, pub->fe->fuzzy) != NULL) {

					if (nk_combo_item_label(ctx, lp->reg[reg_ID].sym,
								NK_TEXT_LEFT)) {

						select_ID = reg_ID;
					}

					pub->fuzzy_count++;
				}
			}
		}

		pub->fuzzy_count = (pub->fuzzy_count > 0) ? pub->fuzzy_count : 1;

		nk_combo_end(ctx);
	}

	return select_ID;
}

static struct nk_rect
pub_get_popup_bounds_tiny(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			win, popup;

	win = nk_window_get_content_region(ctx);

	popup.w = win.w * 0.8f;
	popup.h = win.h * 0.5f;

	popup.x = win.w / 2.f - popup.w / 2.f;
	popup.y = win.h * 0.4f;

	return popup;
}

static struct nk_rect
pub_get_popup_bounds_full(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			win, popup;

	win = nk_window_get_content_region(ctx);

	popup.w = win.w;
	popup.h = win.h;

	popup.x = win.w / 2.f - popup.w / 2.f;
	popup.y = win.h / 2.f - popup.h / 2.f;

	return popup;
}

static void
pub_popup_message(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	if (pub->popup_enum != popup)
		return ;

	bounds = pub_get_popup_bounds_tiny(pub);

	if (nk_popup_begin(ctx, NK_POPUP_DYNAMIC, " ",
				NK_WINDOW_CLOSABLE
				| NK_WINDOW_NO_SCROLLBAR, bounds)) {

		nk_layout_row_template_begin(ctx, pub->fe_font_h * 5);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);
		nk_label_wrap(ctx, title);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_variable(ctx, 1);
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

static void
pub_popup_progress(struct public *pub, int popup, const char *title, int pce)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	if (pub->popup_enum != popup)
		return ;

	bounds = pub_get_popup_bounds_tiny(pub);

	if (nk_popup_begin(ctx, NK_POPUP_DYNAMIC, title,
				NK_WINDOW_CLOSABLE
				| NK_WINDOW_NO_SCROLLBAR, bounds)) {

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);
		nk_prog(ctx, pce, 1000, nk_false);
		nk_spacer(ctx);

		if (pce >= 1000) {

			nk_popup_close(ctx);

			pub->popup_enum = 0;
		}

		nk_popup_end(ctx);
	}
	else {
		pub->popup_enum = 0;
	}
}

static int
pub_popup_ok_cancel(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;

	int				rc = 0;

	if (pub->popup_enum != popup)
		return 0;

	bounds = pub_get_popup_bounds_tiny(pub);

	if (nk_popup_begin(ctx, NK_POPUP_DYNAMIC, " ",
				NK_WINDOW_CLOSABLE
				| NK_WINDOW_NO_SCROLLBAR, bounds)) {

		nk_layout_row_template_begin(ctx, pub->fe_font_h * 5);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);
		nk_label_wrap(ctx, title);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_variable(ctx, 1);
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

	return rc;
}

static void
pub_directory_scan(struct public *pub, const char *sup)
{
	int			len, ofs, bsz, kmg, N;

	if (dirent_open(&pub->scan.sb, pub->fe->storage) != ENT_OK)
		return ;

	len = strlen(sup);
	N = 0;

	while (dirent_read(&pub->scan.sb) == ENT_OK) {

		if (pub->scan.sb.ntype == ENT_TYPE_REGULAR) {

			ofs = strlen(pub->scan.sb.name) - len;

			if (memcmp(pub->scan.sb.name + ofs, sup, len) == 0) {

				strcpy(pub->scan.file[N].name, pub->scan.sb.name);
				strcpy(pub->scan.file[N].time, pub->scan.sb.time);

				bsz = pub->scan.sb.nsize;
				kmg = 0;

				while (bsz >= 1024U) { bsz /= 1024U; ++kmg; }

				sprintf(pub->scan.file[N].size, "%4d %cb",
						(int) bsz, " KMG??" [kmg]);

				++N;

				if (N >= PHOBIA_FILE_MAX - 1)
					break;
			}
		}
	}

	dirent_close(&pub->scan.sb);

	pub->scan.file[N].name[0] = 0;
	pub->scan.selected = - 1;
}

static void
pub_open_GP(struct public *pub, const char *file)
{
	if (pub->gp != NULL) {

		gp_Clean(pub->gp);
	}

	pub->gp = gp_Alloc();

	sprintf(pub->lbuf,	"chunk 10\n"
				"timeout 10000\n"
				"load 0 0 text \"%s\"\n"
				"mkpages 0\n", file);

	gp_TakeConfig(pub->gp, pub->lbuf);

	pub->gp_ID = gp_OpenWindow(pub->gp);
}

static void
pub_popup_telemetry_flush(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;
	struct nk_style_button		disabled;

	int				height;
	int				N, sel, newsel;

	if (pub->popup_enum != popup)
		return ;

	bounds = pub_get_popup_bounds_full(pub);

	if (nk_popup_begin(ctx, NK_POPUP_STATIC, title,
				NK_WINDOW_CLOSABLE
				| NK_WINDOW_NO_SCROLLBAR, bounds)) {

		struct link_reg			*reg_tlm;

		reg_tlm = link_reg_lookup(pub->lp, "tlm.mode");

		if (reg_tlm != NULL) {

			reg_tlm->update = (reg_tlm->lval != 0) ? 100 : 1000;
			reg_tlm->shown = lp->clock;
		}

		nk_layout_row_dynamic(ctx, pub->fe_base, 1);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 10);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Grab into RAM")) {

			link_command(lp, "tlm_grab");

			if (reg_tlm != NULL) {

				reg_tlm->lval = 1;
			}
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Watch on PM")) {

			link_command(lp, "tlm_watch");

			if (reg_tlm != NULL) {

				reg_tlm->lval = 1;
			}
		}

		nk_spacer(ctx);

		if (reg_tlm != NULL && reg_tlm->lval == 0) {

			if (nk_button_label(ctx, "Flush GP")) {

				strcpy(pub->lbuf, pub->fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, pub->telemetry.file_grab);
				strcpy(pub->telemetry.file_snap, pub->lbuf);

				if (link_grab_file_open(lp, pub->telemetry.file_snap) != 0) {

					if (link_command(lp, "tlm_flush_sync") != 0) {

						pub->telemetry.wait_GP = 1;
					}

					pub_directory_scan(pub, FILE_TELEMETRY_FILTER);
				}
			}
		}
		else {
			disabled = ctx->style.button;

			disabled.normal = disabled.active;
			disabled.hover = disabled.active;
			disabled.text_normal = disabled.text_active;
			disabled.text_hover = disabled.text_active;

			nk_button_label_styled(ctx, &disabled, "Flush GP");
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Live GP")) {

			strcpy(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, pub->telemetry.file_grab);
			strcpy(pub->telemetry.file_snap, pub->lbuf);

			if (link_grab_file_open(lp, pub->telemetry.file_snap) != 0) {

				if (link_command(lp, "tlm_live_sync") != 0) {

					pub->telemetry.wait_GP = 1;
				}

				if (reg_tlm != NULL) {

					reg_tlm->lval = 1;
				}

				pub_directory_scan(pub, FILE_TELEMETRY_FILTER);
			}
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, pub->fe_base, 1);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 5);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);

		sprintf(pub->lbuf, "# %i", lp->grab_N);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD,
				pub->telemetry.file_grab,
				sizeof(pub->telemetry.file_grab),
				nk_filter_default);

		if (		pub->scan.selected >= 0
				&& pub->scan.selected < PHOBIA_FILE_MAX) {

			N = pub->scan.selected;

			if (		strcmp(pub->scan.file[N].name,
						pub->telemetry.file_grab) != 0) {

				pub->scan.selected = -1;
			}
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		height = ctx->current->bounds.h - ctx->current->layout->row.height * 8;

		nk_layout_row_template_begin(ctx, height);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);

		if (nk_group_begin(ctx, "FILES", NK_WINDOW_BORDER)) {

			nk_layout_row_template_begin(ctx, 0);
			nk_layout_row_template_push_variable(ctx, 1);
			nk_layout_row_template_push_static(ctx, pub->fe_base * 10);
			nk_layout_row_template_push_static(ctx, pub->fe_base * 5);
			nk_layout_row_template_end(ctx);

			for (N = 0; N < PHOBIA_FILE_MAX; ++N) {

				if (pub->scan.file[N].name[0] == 0)
					break;

				sel = (N == pub->scan.selected) ? 1 : 0;

				newsel = nk_select_label(ctx, pub->scan.file[N].name,
						NK_TEXT_LEFT, sel);

				newsel |= nk_select_label(ctx, pub->scan.file[N].time,
						NK_TEXT_LEFT, sel);

				newsel |= nk_select_label(ctx, pub->scan.file[N].size,
						NK_TEXT_RIGHT, sel);

				if (newsel != sel && newsel != 0) {

					pub->scan.selected = N;

					strcpy(pub->telemetry.file_grab,
							pub->scan.file[N].name);
				}
			}

			nk_group_end(ctx);
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, pub->fe_base, 1);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Scan")) {

			pub_directory_scan(pub, FILE_TELEMETRY_FILTER);
		}

		nk_spacer(ctx);

		if (		pub->scan.selected >= 0
				&& pub->scan.selected < PHOBIA_FILE_MAX) {

			if (nk_button_label(ctx, "Plot GP")) {

				strcpy(pub->lbuf, pub->fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, pub->telemetry.file_grab);
				strcpy(pub->telemetry.file_snap, pub->lbuf);

				pub_open_GP(pub, pub->telemetry.file_snap);
			}

			nk_spacer(ctx);

			if (nk_button_label(ctx, "Remove")) {

				strcpy(pub->lbuf, pub->fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, pub->telemetry.file_grab);

				file_remove(pub->lbuf);

				pub_directory_scan(pub, FILE_TELEMETRY_FILTER);
			}
		}
		else {
			disabled = ctx->style.button;

			disabled.normal = disabled.active;
			disabled.hover = disabled.active;
			disabled.text_normal = disabled.text_active;
			disabled.text_hover = disabled.text_active;

			nk_button_label_styled(ctx, &disabled, "Plot GP");

			nk_spacer(ctx);

			nk_button_label_styled(ctx, &disabled, "Remove");
		}

		nk_spacer(ctx);

		if (		pub->telemetry.wait_GP != 0
				&& lp->grab_N >= 4) {

			pub->telemetry.wait_GP = 0;

			pub_open_GP(pub, pub->telemetry.file_snap);
		}

		nk_popup_end(ctx);
	}
	else {
		if (lp->grab_N != 0) {

			link_grab_file_close(lp);
		}

		pub->popup_enum = 0;
	}
}

static void
pub_drawing_motor_position(struct public *pub, float Fg, int dtype)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct nk_command_buffer	*canvas = nk_window_get_canvas(ctx);
	struct nk_rect			space;

	float				tfm[4], arrow[10], pbuf[10];
	float				radius, thickness;
	int				N, Zp = 0;

	if (nk_widget(&space, ctx) == NK_WIDGET_INVALID)
		return ;

	radius = space.w / 2.f;
	thickness = radius / 10.f;

	space.x += thickness;
	space.y += thickness;
	space.w += - 2.f * thickness;
	space.h += - 2.f * thickness;

	nk_fill_circle(canvas, space, nk->table[NK_COLOR_DRAWING_PEN]);

	space.x += thickness;
	space.y += thickness;
	space.w += - 2.f * thickness;
	space.h += - 2.f * thickness;

	nk_fill_circle(canvas, space, nk->table[NK_COLOR_WINDOW]);

	tfm[0] = space.x + space.w / 2.f;
	tfm[1] = space.y + space.h / 2.f;

	if (dtype == DRAWING_WITH_TOOTH) {

		struct link_reg		*reg;

		reg = link_reg_lookup(pub->lp, "pm.const_Zp");
		if (reg != NULL) { Zp = reg->lval; }

		if (Zp != 0) {

			arrow[0] = radius - 3.f * thickness;
			arrow[1] = - .5f * thickness;
			arrow[2] = radius - 3.f * thickness;
			arrow[3] = .5f * thickness;
			arrow[4] = radius - 1.5f * thickness;
			arrow[5] = .5f * thickness;
			arrow[6] = radius - 1.5f * thickness;
			arrow[7] = - .5f * thickness;
		}
	}

	for (N = 0; N < Zp; ++N) {

		float		Za = (float) N * (float) (2. * M_PI) / (float) Zp;

		tfm[2] = cosf(Za);
		tfm[3] = - sinf(Za);

		pbuf[0] = tfm[0] + (tfm[2] * arrow[0] - tfm[3] * arrow[1]);
		pbuf[1] = tfm[1] + (tfm[3] * arrow[0] + tfm[2] * arrow[1]);
		pbuf[2] = tfm[0] + (tfm[2] * arrow[2] - tfm[3] * arrow[3]);
		pbuf[3] = tfm[1] + (tfm[3] * arrow[2] + tfm[2] * arrow[3]);
		pbuf[4] = tfm[0] + (tfm[2] * arrow[4] - tfm[3] * arrow[5]);
		pbuf[5] = tfm[1] + (tfm[3] * arrow[4] + tfm[2] * arrow[5]);
		pbuf[6] = tfm[0] + (tfm[2] * arrow[6] - tfm[3] * arrow[7]);
		pbuf[7] = tfm[1] + (tfm[3] * arrow[6] + tfm[2] * arrow[7]);

		nk_fill_polygon(canvas, pbuf, 4, nk->table[NK_COLOR_DRAWING_PEN]);
	}

	if (dtype == DRAWING_WITH_HALL) {

		struct nk_color		col;
		struct link_reg		*reg;
		int			HS = -1;

		reg = link_reg_lookup(pub->lp, "pm.fb_HS");
		if (reg != NULL) { HS = reg->lval; }

		arrow[0] = radius - 2.f * thickness;
		arrow[1] = - thickness;
		arrow[2] = radius - 2.f * thickness;
		arrow[3] = thickness;
		arrow[4] = radius;
		arrow[5] = thickness;
		arrow[6] = radius;
		arrow[7] = - thickness;

		for (N = 1; N <= 6; ++N) {

			sprintf(pub->lbuf, "pm.hall_ST%i", N);

			reg = link_reg_lookup(pub->lp, pub->lbuf);

			if (reg == NULL)
				break;

			if ((reg->mode & LINK_REG_TYPE_FLOAT) == 0)
				break;

			tfm[2] = cosf(reg->fval * (float) (M_PI / 180.));
			tfm[3] = - sinf(reg->fval * (float) (M_PI / 180.));

			pbuf[0] = tfm[0] + (tfm[2] * arrow[0] - tfm[3] * arrow[1]);
			pbuf[1] = tfm[1] + (tfm[3] * arrow[0] + tfm[2] * arrow[1]);
			pbuf[2] = tfm[0] + (tfm[2] * arrow[2] - tfm[3] * arrow[3]);
			pbuf[3] = tfm[1] + (tfm[3] * arrow[2] + tfm[2] * arrow[3]);
			pbuf[4] = tfm[0] + (tfm[2] * arrow[4] - tfm[3] * arrow[5]);
			pbuf[5] = tfm[1] + (tfm[3] * arrow[4] + tfm[2] * arrow[5]);
			pbuf[6] = tfm[0] + (tfm[2] * arrow[6] - tfm[3] * arrow[7]);
			pbuf[7] = tfm[1] + (tfm[3] * arrow[6] + tfm[2] * arrow[7]);

			col = (N == HS) ? nk->table[NK_COLOR_ENABLED]
					: nk->table[NK_COLOR_HIDDEN];

			nk_fill_polygon(canvas, pbuf, 4, col);
		}
	}

	tfm[2] = cosf(Fg * (float) (M_PI / 180.));
	tfm[3] = - sinf(Fg * (float) (M_PI / 180.));

	arrow[0] = - .5f * thickness;
	arrow[1] = - .5f * thickness;
	arrow[2] = - .5f * thickness;
	arrow[3] = .5f * thickness;
	arrow[4] = radius - 3.f * thickness;
	arrow[5] = .5f * thickness;
	arrow[6] = radius - 2.f * thickness;
	arrow[7] = 0.f;
	arrow[8] = radius - 3.f * thickness;
	arrow[9] = - .5f * thickness;

	pbuf[0] = tfm[0] + (tfm[2] * arrow[0] - tfm[3] * arrow[1]);
	pbuf[1] = tfm[1] + (tfm[3] * arrow[0] + tfm[2] * arrow[1]);
	pbuf[2] = tfm[0] + (tfm[2] * arrow[2] - tfm[3] * arrow[3]);
	pbuf[3] = tfm[1] + (tfm[3] * arrow[2] + tfm[2] * arrow[3]);
	pbuf[4] = tfm[0] + (tfm[2] * arrow[4] - tfm[3] * arrow[5]);
	pbuf[5] = tfm[1] + (tfm[3] * arrow[4] + tfm[2] * arrow[5]);
	pbuf[6] = tfm[0] + (tfm[2] * arrow[6] - tfm[3] * arrow[7]);
	pbuf[7] = tfm[1] + (tfm[3] * arrow[6] + tfm[2] * arrow[7]);
	pbuf[8] = tfm[0] + (tfm[2] * arrow[8] - tfm[3] * arrow[9]);
	pbuf[9] = tfm[1] + (tfm[3] * arrow[8] + tfm[2] * arrow[9]);

	nk_fill_polygon(canvas, pbuf, 5, nk->table[NK_COLOR_EDIT_NUMBER]);
}

static void
pub_drawing_brick_colored(struct nk_sdl *nk, const int sym)
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

		pub_name_label(pub, name, reg);

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

		pub_name_label_hidden(pub, name, sym);
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

		pub_name_label(pub, name, reg);
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

		pub_name_label_hidden(pub, name, sym);
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
reg_enum_combo(struct public *pub, const char *sym, const char *name, int onlight)
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

		pub_name_label(pub, name, reg);

		reg->lval = (reg->lval < reg->lmax_combo + 2
				&& reg->lval >= 0) ? reg->lval : 0;

		if (		onlight != 0
				&& reg->lval != 0) {

			struct nk_color		normal, hover;

			normal = nk->table[NK_COLOR_SELECT_ACTIVE];
			hover  = nk->table[NK_COLOR_ACTIVE_HOVER];

			ctx->style.combo.normal = nk_style_item_color(normal);
			ctx->style.combo.hover  = nk_style_item_color(hover);
			ctx->style.combo.active = nk_style_item_color(normal);
			ctx->style.combo.button.normal = ctx->style.combo.normal;
			ctx->style.combo.button.hover  = ctx->style.combo.hover;
			ctx->style.combo.button.active = ctx->style.combo.active;
		}

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

		pub_name_label_hidden(pub, name, sym);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

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

		pub_name_label(pub, name, reg);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				reg->val, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		pub_name_label_hidden(pub, name, sym);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

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

		pub_name_label(pub, name, reg);

		if (reg->mode & LINK_REG_READ_ONLY) {

			nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
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

			sprintf(pub->lbuf, "(%.16s)", reg->um);
			nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);
		}
		else {
			nk_spacer(ctx);
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		pub_name_label_hidden(pub, name, sym);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

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

		pub_name_label(pub, name, reg);

		if (reg->mode & LINK_REG_READ_ONLY) {

			nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
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
			lp->reg[min + lp->reg[min].um_sel].onefetch = 1;
		}

		nk_spacer(ctx);

		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		pub_name_label_hidden(pub, name, sym);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

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

		pub_name_label(pub, name, reg);

		rc = pub_combo_linked(pub, reg, pub->fe_font_h + pub->fe_padding,
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

		pub_name_label_hidden(pub, name, sym);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
	}

	ctx->style.combo = combo;
}

static void
reg_float_prog(struct public *pub, int reg_ID)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg = NULL;

	struct nk_style_edit		edit;
	struct nk_color			hidden;

	int				pce;

	edit = ctx->style.edit;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	if (reg_ID > 0 && reg_ID < lp->reg_MAX_N) {

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

		if (reg->started != 0) {

			pce = (int) (1000.f * (reg->fval - reg->fmin)
					/ (reg->fmax - reg->fmin));
		}
		else {
			pce = 0;
		}

		nk_prog(ctx, pce, 1000, nk_false);
		nk_spacer(ctx);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				reg->val, 79, nk_filter_default);

		nk_spacer(ctx);

		if (reg->um[0] != 0) {

			sprintf(pub->lbuf, "(%.16s)", reg->um);
			nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);
		}
		else {
			nk_spacer(ctx);
		}

		nk_spacer(ctx);

		reg->update = 400;
		reg->always = (lp->tlm_N > 0) ? pub->fe->tlmrate : 0;
		reg->shown = lp->clock;
	}
	else {
		hidden = nk->table[NK_COLOR_HIDDEN];

		nk_prog(ctx, 0, 1000, nk_false);
		nk_spacer(ctx);

		pub->lbuf[0] = 0;

		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
				pub->lbuf, 79, nk_filter_default);

		nk_spacer(ctx);
		nk_label_colored(ctx, "x", NK_TEXT_LEFT, hidden);
		nk_spacer(ctx);
	}

	nk_layout_row_dynamic(ctx, pub->fe_base, 1);
	nk_spacer(ctx);

	ctx->style.edit = edit;
}

static void
page_serial(struct public *pub)
{
	struct config_phobia		*fe = pub->fe;
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_style_button		orange;

	const char			*ls_baudrates[] = {

		"9600", "19200", "57600", "115200"
	};

	const char			*ls_windowsize[] = {

		"900x600", "1200x900", "1600x1200"
	};

	int				rc, select;

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

	if (lp->linked == 0) {

		if (pub->serial.started == 0) {

			const char		*portname;
			int			N;

			serial_enumerate(&pub->serial.list);

			pub->serial.started = 1;
			pub->serial.baudrate = 2;

			for (N = 0; N < pub->serial.list.dnum; ++N) {

				portname = pub->serial.list.name[N];

				if (strcmp(portname, fe->serialport) == 0) {

					pub->serial.selected = N;
					break;
				}
			}
		}

		nk_label(ctx, "Serial port", NK_TEXT_LEFT);
		pub->serial.selected = nk_combo(ctx, pub->serial.list.name,
				pub->serial.list.dnum, pub->serial.selected,
				pub->fe_font_h + pub->fe_padding,
				nk_vec2(pub->fe_base * 13, 400));

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Scan")) {

			serial_enumerate(&pub->serial.list);
		}

		nk_spacer(ctx);

		nk_label(ctx, "Baudrate", NK_TEXT_LEFT);
		pub->serial.baudrate = nk_combo(ctx, ls_baudrates, 4,
				pub->serial.baudrate, pub->fe_font_h
				+ pub->fe_padding, nk_vec2(pub->fe_base * 13, 400));

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Connect")) {

			const char		*portname;
			int			baudrate = 0;

			portname = pub->serial.list.name[pub->serial.selected];
			lk_stoi(&baudrate, ls_baudrates[pub->serial.baudrate]);

			link_open(pub->lp, pub->fe, portname, baudrate, SERIAL_DEFAULT);

			if (pub->lp->linked != 0) {

				strcpy(pub->lbuf, fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, FILE_LINK_LOG);

				link_log_file_open(pub->lp, pub->lbuf);

				strcpy(fe->serialport, portname);
				config_write(pub->fe);

				pub->popup_enum = POPUP_LINK_PROGRESS;
			}
		}

		nk_spacer(ctx);

		nk_label(ctx, "Data bits", NK_TEXT_LEFT);
		sprintf(pub->lbuf, "%s", SERIAL_DEFAULT);
		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
			pub->lbuf, sizeof(pub->lbuf), nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
	}
	else {
		nk_label(ctx, "Serial port", NK_TEXT_LEFT);
		nk_label(ctx, pub->lp->devname, NK_TEXT_LEFT);

		nk_spacer(ctx);

		sprintf(pub->lbuf, "# %i", lp->fetched_N);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		nk_label(ctx, "Baudrate", NK_TEXT_LEFT);
		sprintf(pub->lbuf, "%i", lp->baudrate);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Drop")) {

			if (		lp->reg_MAX_N > 300
					&& lp->reg_MAX_N < LINK_REGS_MAX) {

				fe->regmaxn = lp->reg_MAX_N;
			}

			config_write(pub->fe);

			link_close(pub->lp);
		}

		nk_spacer(ctx);

		nk_label(ctx, "Data bits", NK_TEXT_LEFT);
		sprintf(pub->lbuf, "%s", SERIAL_DEFAULT);
		nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
			pub->lbuf, sizeof(pub->lbuf), nk_filter_default);

		nk_spacer(ctx);
		nk_spacer(ctx);
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 22);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_label(ctx, "Window resolution", NK_TEXT_LEFT);

	select = nk_combo(ctx, ls_windowsize, 3, fe->windowsize, pub->fe_font_h
			+ pub->fe_padding, nk_vec2(pub->fe_base * 22, 400));

	if (select != fe->windowsize) {

		fe->windowsize = select;

		config_write(pub->fe);
		pub_font_layout(pub);
	}

	nk_spacer(ctx);

	nk_label(ctx, "Storage PATH", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, fe->storage,
			sizeof(fe->storage), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_label(ctx, "Linked fuzzy pattern", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, fe->fuzzy,
			sizeof(fe->fuzzy), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_label(ctx, "Telemetry log rate", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, fe->tlmrate_lbuf,
			sizeof(fe->tlmrate_lbuf), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		lk_stoi(&pub->fe->tlmrate, fe->tlmrate_lbuf);
		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_label_colored(ctx, fe->rcfile, NK_TEXT_LEFT,
			nk->table[NK_COLOR_HIDDEN]);

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Default")) {

		pub->popup_enum = POPUP_RESET_DEFAULT;
	}

	nk_spacer(ctx);

	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset all local Phobia"
				" frontend configuration") != 0) {

		config_default(pub->fe);
		config_write(pub->fe);

		pub_font_layout(pub);
	}

	if (pub->popup_enum == POPUP_LINK_PROGRESS) {

		int 		link_pce;

		link_pce = (int) (1000.f * lp->reg_MAX_N / (float) fe->regmaxn);

		if (lp->active + 500 < lp->clock) {

			link_pce = 1000;

			if (		lp->reg_MAX_N > 300
					&& lp->reg_MAX_N < LINK_REGS_MAX) {

				fe->regmaxn = lp->reg_MAX_N;
			}

			config_write(pub->fe);
		}

		pub_popup_progress(pub, POPUP_LINK_PROGRESS,
				"Loading ...", link_pce);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_diagnose(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

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
	reg_float(pub, "pm.const_fb_U", "Battery voltage");
	reg_float(pub, "pm.ad_IA0", "A sensor drift");
	reg_float(pub, "pm.ad_IB0", "B sensor drift");
	reg_float(pub, "pm.ad_IC0", "C sensor drift");
	reg_text_large(pub, "pm.self_BST", "Bootstrap retention time");
	reg_text_large(pub, "pm.self_BM", "Self test result");
	reg_text_large(pub, "pm.self_STDi", "Current noise STD");
	reg_text_large(pub, "pm.self_RMSi", "Current sensor RMS");
	reg_text_large(pub, "pm.self_RMSu", "Voltage sensor RMS");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.ad_IA1", "A sensor scale");
	reg_float(pub, "pm.ad_IB1", "B sensor scale");
	reg_float(pub, "pm.ad_IC1", "C sensor scale");
	reg_float(pub, "pm.ad_UA0", "A voltage offset");
	reg_float(pub, "pm.ad_UA1", "A voltage scale");
	reg_float(pub, "pm.ad_UB0", "B voltage offset");
	reg_float(pub, "pm.ad_UB1", "B voltage scale");
	reg_float(pub, "pm.ad_UC0", "C voltage offset");
	reg_float(pub, "pm.ad_UC1", "C voltage scale");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.tvm_USEABLE", "TVM is in service");
	reg_float(pub, "pm.tvm_FIR_A_tau", "A voltage FIR tau");
	reg_float(pub, "pm.tvm_FIR_B_tau", "B voltage FIR tau");
	reg_float(pub, "pm.tvm_FIR_C_tau", "C voltage FIR tau");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.fb_iA", "A current feedback");
	reg_float(pub, "pm.fb_iB", "B current feedback");
	reg_float(pub, "pm.fb_iC", "C current feedback");
	reg_float(pub, "pm.fb_uA", "A voltage feedback");
	reg_float(pub, "pm.fb_uB", "B voltage feedback");
	reg_float(pub, "pm.fb_uC", "C voltage feedback");
	reg_float(pub, "pm.fb_HS", "Hall sensors feedback");
	reg_float(pub, "pm.fb_EP", "EABI encoder feedback");
	reg_float(pub, "pm.fb_SIN", "SIN analog feedback");
	reg_float(pub, "pm.fb_COS", "COS analog feedback");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "PWM  0%")) {

		link_command(pub->lp,	"hal_PWM_set_Z 0" "\r\n"
					"hal_PWM_set_DC 0");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "PWM 50%")) {

		struct link_reg		*reg;
		int			dc_resolution = 2800;

		reg = link_reg_lookup(pub->lp, "pm.dc_resolution");
		if (reg != NULL) { dc_resolution = reg->lval; }

		sprintf(pub->lbuf,	"hal_PWM_set_Z 0" "\r\n"
					"hal_PWM_set_DC %i", dc_resolution / 2);

		link_command(pub->lp, pub->lbuf);
	}

	nk_spacer(ctx);

	if (lp->unable_warning[0] != 0) {

		strcpy(pub->popup_msg, lp->unable_warning);
		pub->popup_enum = POPUP_UNABLE_WARNING;

		lp->unable_warning[0] = 0;
	}

	pub_popup_message(pub, POPUP_UNABLE_WARNING, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_probe(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	int				config_LU_DRIVE = 1;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 10);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
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

	if (nk_button_label(ctx, "Default")) {

		if (pub->lp->linked != 0) {

			pub->popup_enum = POPUP_RESET_DEFAULT;
		}
	}

	nk_spacer(ctx);

	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset all measured parameters"
				" related to the motor") != 0) {

		link_command(pub->lp, "pm_default_probe");
		link_reg_fetch_all_shown(pub->lp);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_errno(pub, "pm.fsm_errno", "FSM errno", 1);
	reg_float(pub, "pm.const_fb_U", "Battery voltage");
	reg_float_um(pub, "pm.const_lambda", "Flux linkage", 1);
	reg_float(pub, "pm.const_Rs", "Winding resistance");
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

		reg = link_reg_lookup(pub->lp, "pm.const_lambda");
		if (reg != NULL) { reg += reg->um_sel; reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "pm.const_Ja");
		if (reg != NULL) { reg += reg->um_sel; reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Start FSM")) {

		if (link_command(pub->lp, "pm_fsm_startup") != 0) {

			reg = link_reg_lookup(pub->lp, "pm.lu_MODE");
			if (reg != NULL) { reg->onefetch = 1; }

			if (config_LU_DRIVE == 0) {

				reg = link_reg_lookup(pub->lp, "pm.i_setpoint_current");
				if (reg != NULL) { reg += reg->um_sel; reg->onefetch = 1; }
			}
			else if (config_LU_DRIVE == 1) {

				reg = link_reg_lookup(pub->lp, "pm.s_setpoint_speed");
				if (reg != NULL) { reg += reg->um_sel; reg->onefetch = 1; }
			}
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Stop FSM")) {

		if (link_command(pub->lp, "pm_fsm_shutdown") != 0) {

			reg = link_reg_lookup(pub->lp, "pm.lu_MODE");
			if (reg != NULL) { reg->onefetch = 1; }
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);

	reg = link_reg_lookup(pub->lp, "pm.config_LU_DRIVE");
	if (reg != NULL) { config_LU_DRIVE = reg->lval; }

	if (		   config_LU_DRIVE == 0
			|| config_LU_DRIVE == 1) {

		reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

		if (reg != NULL) {

			int		rate;

			reg->update = 1000;

			rate = (reg->lval != 0) ? 100 : 0;

			reg = link_reg_lookup(pub->lp, "pm.lu_iD");
			if (reg != NULL) { reg->update = rate; }

			reg = link_reg_lookup(pub->lp, "pm.lu_iQ");
			if (reg != NULL) { reg->update = rate; }

			reg = link_reg_lookup(pub->lp, "pm.lu_wS");
			if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

			reg = link_reg_lookup(pub->lp, "pm.lu_mq_load");
			if (reg != NULL) { reg->update = rate; }
		}

		reg_float(pub, "pm.lu_iD", "LU current D");
		reg_float(pub, "pm.lu_iQ", "LU current Q");
		reg_float_um(pub, "pm.lu_wS", "LU speed estimate", 1);
		reg_float(pub, "pm.lu_mq_load", "LU torque estimate");

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		if (config_LU_DRIVE == 0) {

			reg_float_um(pub, "pm.i_setpoint_current", "Current setpoint", 0);
		}
		else if (config_LU_DRIVE == 1) {

			reg_float_um(pub, "pm.s_setpoint_speed", "Speed setpoint", 1);
		}

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 11);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 9);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		if (nk_button_label(ctx, "Probe Flux Linkage")) {

			link_command(pub->lp, "pm_probe_const_flux_linkage");
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Probe Inertia")) {

			link_command(pub->lp, "pm_probe_const_inertia");
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Probe Noise")) {

			link_command(pub->lp, "pm_probe_noise_threshold");
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		reg_float_um(pub, "pm.zone_speed_noise", "Noise threshold", 0);
	}

	if (lp->unable_warning[0] != 0) {

		strcpy(pub->popup_msg, lp->unable_warning);
		pub->popup_enum = POPUP_UNABLE_WARNING;

		lp->unable_warning[0] = 0;
	}

	pub_popup_message(pub, POPUP_UNABLE_WARNING, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_hal(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_float(pub, "hal.USART_baud_rate", "USART baudrate");
	reg_float(pub, "hal.PWM_frequency", "PWM frequency");
	reg_float(pub, "hal.PWM_deadtime", "PWM deadtime");
	reg_float(pub, "hal.ADC_reference_voltage", "ADC reference voltage");
	reg_float(pub, "hal.ADC_shunt_resistance", "Current shunt resistance");
	reg_float(pub, "hal.ADC_amplifier_gain", "Current amplifier gain");
	reg_float(pub, "hal.ADC_voltage_ratio", "Battery voltage ratio");
	reg_float(pub, "hal.ADC_terminal_ratio", "Terminal voltage ratio");
	reg_float(pub, "hal.ADC_terminal_bias", "Terminal voltage bias");
	reg_float(pub, "hal.ADC_knob_ratio", "Knob voltage ratio");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "hal.DPS_mode", "DPS operation mode", 0);
	reg_enum_combo(pub, "hal.PPM_mode", "PPM operation mode", 0);
	reg_float(pub, "hal.PPM_timebase", "PPM time base");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "hal.DRV.part", "DRV part", 1);
	reg_enum_toggle(pub, "hal.DRV.auto_RESET", "DRV automatic reset");
	reg_float(pub, "hal.DRV.status_raw", "DRV status raw");
	reg_float(pub, "hal.DRV.gate_current", "DRV gate current");
	reg_float(pub, "hal.DRV.ocp_level", "DRV OCP level");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_in_network(struct public *pub)
{
	struct config_phobia		*fe = pub->fe;
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	struct nk_style_button		orange, disabled;

	int				N, height, sel, newsel;

	orange = ctx->style.button;
	orange.normal = nk_style_item_color(nk->table[NK_COLOR_ORANGE_BUTTON]);
	orange.hover = nk_style_item_color(nk->table[NK_COLOR_ORANGE_BUTTON_HOVER]);

	reg_float(pub, "net.node_ID", "Node network ID");
	reg_enum_combo(pub, "net.log_MODE", "Messages logging", 1);
	reg_float(pub, "net.timeout_EP", "EP timeout");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Survey")) {

		link_command(pub->lp, "net_survey");

		pub->network.selected = -1;
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Assign")) {

		link_command(pub->lp,	"net_assign" "\r\n"
					"net_survey");

		reg = link_reg_lookup(pub->lp, "net.node_ID");
		if (reg != NULL) { reg->onefetch = 1; }

		pub->network.selected = -1;
	}

	nk_spacer(ctx);

	N = pub->network.selected;

	if (		N >= 0 && N < LINK_EPCAN_MAX
			&& lp->epcan[N].UID[0] != 0) {

		if (nk_button_label(ctx, "Revoke")) {

			sprintf(pub->lbuf,	"net_revoke %.7s" "\r\n"
						"net_survey", lp->epcan[N].node_ID);

			link_command(pub->lp, pub->lbuf);

			pub->network.selected = -1;
		}

		nk_spacer(ctx);

		if (strstr(lp->network, "remote") == NULL) {

			if (nk_button_label_styled(ctx, &orange, "Connect")) {

				sprintf(pub->lbuf, "net_node_remote %.7s",
						lp->epcan[N].node_ID);

				link_command(pub->lp, pub->lbuf);
				link_remote(pub->lp);

				pub->network.selected = -1;
				pub->popup_enum = POPUP_LINK_PROGRESS;
			}
		}
		else {
			if (nk_button_label_styled(ctx, &orange, "Drop")) {

				link_command(pub->lp, "\x04");
				link_remote(pub->lp);

				pub->network.selected = -1;
				pub->popup_enum = POPUP_LINK_PROGRESS;
			}
		}
	}
	else {
		disabled = ctx->style.button;

		disabled.normal = disabled.active;
		disabled.hover = disabled.active;
		disabled.text_normal = disabled.text_active;
		disabled.text_hover = disabled.text_active;

		nk_button_label_styled(ctx, &disabled, "Revoke");

		nk_spacer(ctx);

		if (strstr(lp->network, "remote") == NULL) {

			nk_button_label_styled(ctx, &disabled, "Connect");
		}
		else {
			if (nk_button_label_styled(ctx, &orange, "Drop")) {

				link_command(pub->lp, "\x04");
				link_remote(pub->lp);

				pub->network.selected = -1;
				pub->popup_enum = POPUP_LINK_PROGRESS;
			}
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	height = ctx->current->layout->row.height * 5;

	nk_layout_row_template_begin(ctx, height);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "NETWORK", NK_WINDOW_BORDER)) {

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 10);
		nk_layout_row_template_end(ctx);

		for (N = 0; N < LINK_EPCAN_MAX; ++N) {

			if (lp->epcan[N].UID[0] == 0)
				break;

			sel = (N == pub->network.selected) ? 1 : 0;

			newsel = nk_select_label(ctx, lp->epcan[N].UID,
					NK_TEXT_LEFT, sel);

			newsel |= nk_select_label(ctx, lp->epcan[N].node_ID,
					NK_TEXT_LEFT, sel);

			if (newsel != sel && newsel != 0) {

				pub->network.selected = N;
			}
		}

		nk_group_end(ctx);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "net.ep0_MODE", "EP 0 operation mode", 1);

	reg = link_reg_lookup(pub->lp, "net.ep0_MODE");

	if (reg != NULL && reg->lval != 0) {

		reg_float(pub, "net.ep0_ID", "EP 0 network ID");
		reg_float(pub, "net.ep0_clock_ID", "EP 0 clock ID");
		reg_float(pub, "net.ep0_reg_DATA", "EP 0 data");
		reg_linked(pub, "net.ep0_reg_ID", "EP 0 register ID");
		reg_enum_toggle(pub, "net.ep0_STARTUP", "EP 0 startup control");
		reg_float(pub, "net.ep0_TIM", "EP 0 frequency");
		reg_enum_combo(pub, "net.ep0_PAYLOAD", "EP 0 payload type", 0);
		reg_float(pub, "net.ep0_range0", "EP 0 range LOW");
		reg_float(pub, "net.ep0_range1", "EP 0 range HIGH");
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "net.ep1_MODE", "EP 1 operation mode", 1);

	reg = link_reg_lookup(pub->lp, "net.ep1_MODE");

	if (reg != NULL && reg->lval != 0) {

		reg_float(pub, "net.ep1_ID", "EP 1 network ID");
		reg_float(pub, "net.ep1_clock_ID", "EP 1 clock ID");
		reg_float(pub, "net.ep1_reg_DATA", "EP 1 data");
		reg_linked(pub, "net.ep1_reg_ID", "EP 1 register ID");
		reg_enum_toggle(pub, "net.ep1_STARTUP", "EP 1 startup control");
		reg_float(pub, "net.ep1_TIM", "EP 1 frequency");
		reg_enum_combo(pub, "net.ep1_PAYLOAD", "EP 1 payload type", 0);
		reg_float(pub, "net.ep1_range0", "EP 1 range LOW");
		reg_float(pub, "net.ep1_range1", "EP 1 range HIGH");
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "net.ep2_MODE", "EP 2 operation mode", 1);

	reg = link_reg_lookup(pub->lp, "net.ep2_MODE");

	if (reg != NULL && reg->lval != 0) {

		reg_float(pub, "net.ep2_ID", "EP 2 network ID");
		reg_float(pub, "net.ep2_clock_ID", "EP 2 clock ID");
		reg_float(pub, "net.ep2_reg_DATA", "EP 2 data");
		reg_linked(pub, "net.ep2_reg_ID", "EP 2 register ID");
		reg_enum_toggle(pub, "net.ep2_STARTUP", "EP 2 startup control");
		reg_float(pub, "net.ep2_TIM", "EP 2 frequency");
		reg_enum_combo(pub, "net.ep2_PAYLOAD", "EP 2 payload type", 0);
		reg_float(pub, "net.ep2_range0", "EP 2 range LOW");
		reg_float(pub, "net.ep2_range1", "EP 2 range HIGH");
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "net.ep3_MODE", "EP 3 operation mode", 1);

	reg = link_reg_lookup(pub->lp, "net.ep3_MODE");

	if (reg != NULL && reg->lval != 0) {

		reg_float(pub, "net.ep3_ID", "EP 3 network ID");
		reg_float(pub, "net.ep3_clock_ID", "EP 3 clock ID");
		reg_float(pub, "net.ep3_reg_DATA", "EP 3 data");
		reg_linked(pub, "net.ep3_reg_ID", "EP 3 register ID");
		reg_enum_toggle(pub, "net.ep3_STARTUP", "EP 3 startup control");
		reg_float(pub, "net.ep3_TIM", "EP 3 frequency");
		reg_enum_combo(pub, "net.ep3_PAYLOAD", "EP 3 payload type", 0);
		reg_float(pub, "net.ep3_range0", "EP 3 range LOW");
		reg_float(pub, "net.ep3_range1", "EP 3 range HIGH");
	}

	if (pub->popup_enum == POPUP_LINK_PROGRESS) {

		int 		link_pce;

		link_pce = (int) (1000.f * lp->reg_MAX_N / (float) fe->regmaxn);

		if (lp->active + 500 < lp->clock) {

			link_pce = 1000;

			if (		lp->reg_MAX_N > 300
					&& lp->reg_MAX_N < LINK_REGS_MAX) {

				fe->regmaxn = lp->reg_MAX_N;
			}

			config_write(pub->fe);
		}

		pub_popup_progress(pub, POPUP_LINK_PROGRESS,
				"Loading ...", link_pce);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_in_stepdir(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	/* TODO */
}

static void
page_in_pwm(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "hal.PPM_caught");

	if (reg != NULL) {

		int		rate;

		reg->update = 400;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "hal.PPM_get_PERIOD");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "hal.PPM_get_PULSE");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_float(pub, "hal.PPM_caught", "PWM signal caught");
	reg_float(pub, "hal.PPM_get_PERIOD", "PWM period received");
	reg_float(pub, "hal.PPM_get_PULSE", "PWM pulse received");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_linked(pub, "ap.ppm_reg_ID", "Control register ID");
	reg_enum_toggle(pub, "ap.ppm_STARTUP", "Startup control");
	reg_float(pub, "ap.ppm_in_range0", "Input range LOW");
	reg_float(pub, "ap.ppm_in_range1", "Input range MID");
	reg_float(pub, "ap.ppm_in_range2", "Input range HIGH");
	reg_float(pub, "ap.ppm_control_range0", "Control range LOW");
	reg_float(pub, "ap.ppm_control_range1", "Control range MID");
	reg_float(pub, "ap.ppm_control_range2", "Control range HIGH");

	reg = link_reg_lookup(pub->lp, "ap.ppm_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range0");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range1");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range2");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.idle_TIME", "IDLE timeout");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_in_knob(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "hal.ADC_get_knob_ANG");
	if (reg != NULL) { reg->update = 100; }

	reg = link_reg_lookup(pub->lp, "hal.ADC_get_knob_BRK");
	if (reg != NULL) { reg->update = 100; }

	reg_float(pub, "hal.ADC_get_knob_ANG", "ANG voltage");
	reg_float(pub, "hal.ADC_get_knob_BRK", "BRK voltage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "ap.knob_ENABLED", "Knob control");
	reg_linked(pub, "ap.knob_reg_ID", "Control register ID");
	reg_enum_toggle(pub, "ap.knob_STARTUP", "Startup control");
	reg_float(pub, "ap.knob_in_ANG0", "ANG range LOW");
	reg_float(pub, "ap.knob_in_ANG1", "ANG range MID");
	reg_float(pub, "ap.knob_in_ANG2", "ANG range HIGH");
	reg_float(pub, "ap.knob_in_BRK0", "BRK range LOW");
	reg_float(pub, "ap.knob_in_BRK1", "BRK range HIGH");
	reg_float(pub, "ap.knob_in_lost0", "Lost range LOW");
	reg_float(pub, "ap.knob_in_lost1", "Lost range HIGH");
	reg_float(pub, "ap.knob_control_ANG0", "Control range LOW");
	reg_float(pub, "ap.knob_control_ANG1", "Control range MID");
	reg_float(pub, "ap.knob_control_ANG2", "Control range HIGH");
	reg_float(pub, "ap.knob_control_BRK", "BRK control");

	reg = link_reg_lookup(pub->lp, "ap.knob_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG0");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG1");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG2");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_BRK");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.idle_TIME", "IDLE timeout");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_apps(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_enum_toggle(pub, "ap.task_AS5047", "AS5047 magnetic encoder");
	reg_enum_toggle(pub, "ap.task_HX711", "HX711 load cell ADC");
	reg_enum_toggle(pub, "ap.task_MPU6050", "MPU6050 inertial sensor");
	reg_enum_toggle(pub, "ap.task_PUSHTWO", "Two buttons control");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.adc_load_kg", "HX711 measurement");
	reg_float(pub, "ap.adc_load_scale0", "HX711 load offset");
	reg_float(pub, "ap.adc_load_scale1", "HX711 load scale");

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

	reg_enum_combo(pub, "ap.ntc_PCB.type", "NTC type (PCB)", 1);
	reg_float(pub, "ap.ntc_PCB.balance", "NTC balance (PCB)");
	reg_float(pub, "ap.ntc_PCB.ntc0", "NTC resistance at Ta");
	reg_float(pub, "ap.ntc_PCB.ta0", "NTC Ta (PCB)");
	reg_float(pub, "ap.ntc_PCB.betta", "NTC betta (PCB)");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "ap.ntc_EXT.type", "NTC type (EXT)", 1);
	reg_float(pub, "ap.ntc_EXT.balance", "NTC balance (EXT)");
	reg_float(pub, "ap.ntc_EXT.ntc0", "NTC resistance at Ta");
	reg_float(pub, "ap.ntc_EXT.ta0", "NTC Ta (EXT)");
	reg_float(pub, "ap.ntc_EXT.betta", "NTC betta (EXT)");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "ap.temp_PCB");
	if (reg != NULL) { reg->update = 1000; }

	reg = link_reg_lookup(pub->lp, "ap.temp_EXT");
	if (reg != NULL) { reg->update = 1000; }

	reg = link_reg_lookup(pub->lp, "ap.temp_MCU");
	if (reg != NULL) { reg->update = 1000; }

	reg_float(pub, "ap.temp_PCB", "Temperature PCB");
	reg_float(pub, "ap.temp_EXT", "Temperature EXT");
	reg_float(pub, "ap.temp_MCU", "Temperature MCU");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.tpro_PCB_temp_halt", "Halt threshold (PCB)");
	reg_float(pub, "ap.tpro_PCB_temp_derate", "Derate threshold (PCB)");
	reg_float(pub, "ap.tpro_PCB_temp_FAN", "Fan ON threshold (PCB)");
	reg_float(pub, "ap.tpro_EXT_temp_derate", "Derate threshold (EXT)");
	reg_float(pub, "ap.tpro_derated_PCB", "Derated current (PCB)");
	reg_float(pub, "ap.tpro_derated_EXT", "Derated current (EXT)");
	reg_float(pub, "ap.tpro_temp_recovery", "Thermal recovery hysteresis");

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
	struct link_reg			*reg;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Default")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_RESET_DEFAULT;
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.dc_resolution", "PWM resolution");
	reg_float(pub, "pm.dc_minimal", "Minimal pulse");
	reg_float(pub, "pm.dc_clearance", "Clearance before ADC sample");
	reg_float(pub, "pm.dc_skip", "Skip after ADC sample");
	reg_float(pub, "pm.dc_bootstrap", "Bootstrap retention time");
	reg_float(pub, "pm.dc_clamped", "Bootstrap recharge time");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "pm.config_NOP", "Number of machine phases", 0);
	reg_enum_combo(pub, "pm.config_IFB", "Current measurement scheme", 0);
	reg_enum_toggle(pub, "pm.config_TVM", "Terminal voltage measurement");
	reg_enum_combo(pub, "pm.config_SALIENCY", "Machine saliency", 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.config_VSI_CIRCULAR", "Circular SVPWM clamping");
	reg_enum_toggle(pub, "pm.config_VSI_PRECISE", "Precise SVPWM in middle");
	reg_enum_toggle(pub, "pm.config_LU_FORCED", "Forced control");
	reg_enum_combo(pub, "pm.config_LU_ESTIMATE", "Sensorless estimate", 1);
	reg_enum_combo(pub, "pm.config_LU_SENSOR", "Rotor position SENSOR", 1);
	reg_enum_combo(pub, "pm.config_LU_LOCATION", "Servo LOCATION source", 1);
	reg_enum_combo(pub, "pm.config_LU_DRIVE", "Drive control loop", 0);

	reg = link_reg_lookup(pub->lp, "pm.config_LU_ESTIMATE");

	if (reg != NULL && reg->lval == 2) {

		reg_enum_combo(pub, "pm.config_HFI_WAVETYPE", "HFI waveform type", 1);
		reg_enum_toggle(pub, "pm.config_HFI_POLARITY", "HFI flux polarity");
	}

	reg_enum_toggle(pub, "pm.config_RELUCTANCE", "Reluctance MTPA control");
	reg_enum_toggle(pub, "pm.config_WEAKENING", "Flux weakening control");

	reg = link_reg_lookup(pub->lp, "pm.config_LU_DRIVE");

	if (reg != NULL && reg->lval == 0) {

		reg_enum_toggle(pub, "pm.config_HOLDING_BRAKE", "Holding brake");
		reg_enum_toggle(pub, "pm.config_SPEED_LIMITED", "Speed limited");
	}

	reg_enum_combo(pub, "pm.config_EABI_FRONTEND", "EABI frontend", 0);
	reg_enum_combo(pub, "pm.config_SINCOS_FRONTEND", "SINCOS frontend", 0);
	reg_enum_toggle(pub, "pm.config_MILEAGE_INFO", "Mileage info");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.tm_transient_slow", "Transient time slow");
	reg_float(pub, "pm.tm_transient_fast", "Transient time fast");
	reg_float(pub, "pm.tm_voltage_hold", "Voltage hold time");
	reg_float(pub, "pm.tm_current_hold", "Current hold time");
	reg_float(pub, "pm.tm_current_ramp", "Current ramp time");
	reg_float(pub, "pm.tm_instant_probe", "Instant probe time");
	reg_float(pub, "pm.tm_average_probe", "Average probe time");
	reg_float(pub, "pm.tm_average_drift", "Average drift time");
	reg_float(pub, "pm.tm_average_inertia", "Average inertia time");
	reg_float(pub, "pm.tm_startup", "Startup time");
	reg_float(pub, "pm.tm_halt_pause", "Halt pause");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.probe_current_hold", "Probe hold current");
	reg_float(pub, "pm.probe_current_weak", "Probe weak current");
	reg_float(pub, "pm.probe_hold_angle", "Probe hold angle");
	reg_float(pub, "pm.probe_current_sine", "Probe sine current");
	reg_float(pub, "pm.probe_current_bias", "Probe bias current");
	reg_float(pub, "pm.probe_freq_sine", "Probe sine frequency");
	reg_float_um(pub, "pm.probe_speed_hold", "Probe speed", 0);
	reg_float_um(pub, "pm.probe_speed_detached", "Probe speed (detached)", 0);
	reg_float(pub, "pm.probe_damping_current", "Damping of current loop");
	reg_float(pub, "pm.probe_damping_speed", "Damping of speed loop");
	reg_float_um(pub, "pm.probe_speed_tol", "Speed settle tolerance", 0);
	reg_float_um(pub, "pm.probe_location_tol", "Location settle tolerance", 0);
	reg_float(pub, "pm.probe_gain_P", "Probe loop gain P");
	reg_float(pub, "pm.probe_gain_I", "Probe loop gain I");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.fault_voltage_tol", "Voltage tolerance");
	reg_float(pub, "pm.fault_current_tol", "Current tolerance");
	reg_float(pub, "pm.fault_accuracy_tol", "Accuracy tolerance");
	reg_float(pub, "pm.fault_terminal_tol", "Terminal tolerance");
	reg_float(pub, "pm.fault_current_halt", "Current halt threshold");
	reg_float(pub, "pm.fault_voltage_halt", "Voltage halt threshold");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.vsi_gain_LP", "VSI gain LP");
	reg_enum_combo(pub, "pm.vsi_mask_XF", "VSI channel mask", 1);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.tvm_USEABLE", "TVM is in service");
	reg_float(pub, "pm.tvm_clean_zone", "TVM clean zone");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.lu_rate", "LU transient rate");
	reg_float(pub, "pm.lu_gain_mq_LP", "LU torque gain LP");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset all PMC configuration") != 0) {

		link_command(pub->lp, "pm_default_config");
		link_reg_fetch_all_shown(pub->lp);
	}

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
	reg_float(pub, "pm.forced_slew_rate", "Current slew rate");
	reg_float(pub, "pm.forced_maximal_DC", "Maximal DC usage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_lu_flux(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "pm.detach_voltage", "Detached threshold");
	reg_float(pub, "pm.detach_trip_AP", "Detached trip point");
	reg_float(pub, "pm.detach_gain_SF", "Detached speed loop gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.flux_trip_AP", "Ortega speed trip point");
	reg_float(pub, "pm.flux_gain_IN", "Ortega initial gain");
	reg_float(pub, "pm.flux_gain_LO", "Ortega flux gain LO");
	reg_float(pub, "pm.flux_gain_HI", "Ortega flux gain HI");
	reg_float(pub, "pm.flux_gain_SF", "Ortega speed loop gain");
	reg_float(pub, "pm.flux_gain_IF", "Torque acceleration");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.kalman_gain_Q0", "Kalman Q0 current D");
	reg_float(pub, "pm.kalman_gain_Q1", "Kalman Q1 current Q");
	reg_float(pub, "pm.kalman_gain_Q2", "Kalman Q2 position");
	reg_float(pub, "pm.kalman_gain_Q3", "Kalman Q3 speed");
	reg_float(pub, "pm.kalman_gain_Q4", "Kalman Q4 bias Q");
	reg_float(pub, "pm.kalman_gain_R", "Kalman R gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_um(pub, "pm.zone_speed_noise", "Noise threshold", 0);
	reg_float_um(pub, "pm.zone_speed_threshold", "Zone threshold", 0);
	reg_float(pub, "pm.zone_gain_TH", "Zone hysteresis");
	reg_float(pub, "pm.zone_gain_LP", "Zone gain LP");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate;

		reg->update = 1000;

		rate = (reg->lval != 0) ? 200 : 0;

		reg = link_reg_lookup(pub->lp, "pm.flux_ZONE");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.flux_wS");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.config_LU_ESTIMATE");

		if (		reg != NULL
				&& reg->lval == 1) {

			reg = link_reg_lookup(pub->lp, "pm.flux_lambda");
			if (reg != NULL) { reg->update = rate; }

			reg = link_reg_lookup(pub->lp, "pm.kalman_bias_Q");
			if (reg != NULL) { reg->update = 0; }
		}

		if (		reg != NULL
				&& reg->lval == 2) {

			reg = link_reg_lookup(pub->lp, "pm.flux_lambda");
			if (reg != NULL) { reg->update = 0; }

			reg = link_reg_lookup(pub->lp, "pm.kalman_bias_Q");
			if (reg != NULL) { reg->update = rate; }
		}
	}

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);
	reg_enum_errno(pub, "pm.flux_ZONE", "FLUX speed ZONE", 0);
	reg_float_um(pub, "pm.flux_wS", "FLUX speed estimate", 0);
	reg_float(pub, "pm.flux_lambda", "Flux linkage estimate");
	reg_float(pub, "pm.kalman_bias_Q", "Q relaxation bias");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lu_hfi(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	int				m_drawing = pub->fe_def_size_x / 4;

	reg_float(pub, "pm.hfi_freq", "HF injection frequency");
	reg_float(pub, "pm.hfi_sine", "HF injection current");
	reg_float(pub, "pm.hfi_gain_DP", "Flux polarity gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate, fast;

		reg->update = 1000;

		rate = (reg->lval != 0) ? 400 : 0;
		fast = (reg->lval != 0) ? 20  : 0;

		reg = link_reg_lookup(pub->lp, "pm.flux_F_g");
		if (reg != NULL) { reg->update = fast; }

		reg = link_reg_lookup(pub->lp, "pm.flux_wS");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.hfi_pole");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);
	reg_float(pub, "pm.flux_F_g", "FLUX position");
	reg_float_um(pub, "pm.flux_wS", "FLUX speed estimate", 1);
	reg_float(pub, "pm.hfi_pole", "HFI flux polarity");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, m_drawing + 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, m_drawing + 20);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "MOTOR", NK_WINDOW_BORDER)) {

		nk_layout_row_static(ctx, m_drawing, m_drawing, 1);

		reg = link_reg_lookup(pub->lp, "pm.flux_F_g");

		if (		reg != NULL
				&& (reg->mode & LINK_REG_TYPE_FLOAT) != 0) {

			pub_drawing_motor_position(pub, reg->fval, DRAWING_EMPTY);
		}

		nk_group_end(ctx);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lu_hall(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	int				m_drawing = pub->fe_def_size_x / 4;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Adjust Hall")) {

		link_command(pub->lp, "pm_adjust_sensor_hall");
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.hall_ST1", "Hall ST 1");
	reg_float(pub, "pm.hall_ST2", "Hall ST 2");
	reg_float(pub, "pm.hall_ST3", "Hall ST 3");
	reg_float(pub, "pm.hall_ST4", "Hall ST 4");
	reg_float(pub, "pm.hall_ST5", "Hall ST 5");
	reg_float(pub, "pm.hall_ST6", "Hall ST 6");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.hall_USEABLE", "Hall is in service");
	reg_float(pub, "pm.hall_trip_AP", "Hall speed trip point");
	reg_float(pub, "pm.hall_gain_LO", "Hall speed gain LO");
	reg_float(pub, "pm.hall_gain_SF", "Hall speed loop gain");
	reg_float(pub, "pm.hall_gain_IF", "Torque acceleration");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate, fast;

		reg->update = 1000;

		rate = (reg->lval != 0) ? 400 : 0;
		fast = (reg->lval != 0) ? 20  : 0;

		reg = link_reg_lookup(pub->lp, "pm.hall_F_g");
		if (reg != NULL) { reg->update = fast; }

		reg = link_reg_lookup(pub->lp, "pm.hall_wS");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		rate = (rate != 0) ? rate : 1000;

		reg = link_reg_lookup(pub->lp, "pm.fb_HS");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);
	reg_float(pub, "pm.hall_F_g", "Hall position");
	reg_float_um(pub, "pm.hall_wS", "Hall speed estimate", 1);
	reg_float(pub, "pm.fb_HS", "Hall feedback");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, m_drawing + 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, m_drawing + 20);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "MOTOR", NK_WINDOW_BORDER)) {

		nk_layout_row_static(ctx, m_drawing, m_drawing, 1);

		reg = link_reg_lookup(pub->lp, "pm.hall_F_g");

		if (		reg != NULL
				&& (reg->mode & LINK_REG_TYPE_FLOAT) != 0) {

			pub_drawing_motor_position(pub, reg->fval, DRAWING_WITH_HALL);
		}

		nk_group_end(ctx);
	}

	if (lp->unable_warning[0] != 0) {

		strcpy(pub->popup_msg, lp->unable_warning);
		pub->popup_enum = POPUP_UNABLE_WARNING;

		lp->unable_warning[0] = 0;
	}

	pub_popup_message(pub, POPUP_UNABLE_WARNING, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lu_eabi(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	/* TODO */
}

static void
page_lu_sincos(struct public *pub)
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
	reg_float(pub, "pm.watt_wA_maximal", "Maximal battery current");
	reg_float(pub, "pm.watt_wP_reverse", "Maximal regeneration");
	reg_float(pub, "pm.watt_wA_reverse", "Maximal battery reverse");
	reg_float(pub, "pm.watt_uDC_maximal", "High battery voltage");
	reg_float(pub, "pm.watt_uDC_minimal", "Low battery voltage");
	reg_float(pub, "pm.watt_gain_LP", "Wattage gain LP");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate;

		reg->update = 1000;
		reg->shown = pub->lp->clock;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "pm.const_fb_U");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.watt_consumption_wP");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		rate = (rate != 0) ? 1000 : 0;

		reg = link_reg_lookup(pub->lp, "pm.lu_total_revol");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.mi_traveled");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.mi_consumed_Wh");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.mi_reverted_Wh");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.mi_fuel_gauge");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_float(pub, "pm.const_fb_U", "Battery voltage");
	reg_float_um(pub, "pm.watt_consumption", "Battery consumption", 1);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.lu_total_revol", "Total electrical revolutions");
	reg_float_um(pub, "pm.mi_traveled", "Total distance traveled", 1);
	reg_float_um(pub, "pm.mi_consumed", "Total consumed energy", 0);
	reg_float_um(pub, "pm.mi_reverted", "Total reverted energy", 0);
	reg_float(pub, "pm.mi_capacity_Ah", "Battery full capacity");
	reg_float(pub, "pm.mi_fuel_gauge", "Fuel gauge");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lp_current(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float(pub, "pm.i_derate_on_HFI", "Derate on HFI");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_um(pub, "pm.i_setpoint_current", "Current setpoint", 0);
	reg_float(pub, "pm.i_maximal", "Maximal forward current");
	reg_float(pub, "pm.i_reverse", "Maximal reverse current");
	reg_float(pub, "pm.i_slew_rate", "Slew rate");
	reg_float(pub, "pm.i_tolerance", "Tolerance");
	reg_float(pub, "pm.i_gain_P", "Proportional loop gain");
	reg_float(pub, "pm.i_gain_I", "Integral loop gain");

	reg = link_reg_lookup(pub->lp, "pm.i_maximal");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.i_reverse");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	reg = link_reg_lookup(pub->lp, "pm.i_gain_P");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.i_slew_rate_D");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "pm.i_slew_rate_Q");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "pm.i_gain_I");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.weak_maximal", "Maximal weakening");
	reg_float(pub, "pm.weak_gain_EU", "Weak regulation gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.v_maximal", "Maximal forward voltage");
	reg_float(pub, "pm.v_reverse", "Maximal reverse voltage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lp_speed(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float_um(pub, "pm.s_setpoint_speed", "Speed setpoint", 0);
	reg_float_um(pub, "pm.s_maximal", "Maximal forward speed", 0);
	reg_float_um(pub, "pm.s_reverse", "Maximal reverse speed", 0);
	reg_float_um(pub, "pm.s_accel", "Maximal acceleration", 0);
	reg_float(pub, "pm.s_linspan", "Regulation span");
	reg_float(pub, "pm.s_tolerance", "Tolerance");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.lu_gain_mq_LP", "LU torque gain LP");
	reg_float(pub, "pm.s_gain_P", "Proportional loop gain");
	reg_float(pub, "pm.s_gain_I", "Forward loop gain");

	reg = link_reg_lookup(pub->lp, "pm.s_gain_P");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.lu_gain_mq_LP");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "pm.s_gain_I");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_lp_location(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	/* TODO */
}

static void
page_telemetry(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Grab")) {

		strcpy(pub->popup_msg, " Telemetry Flush");
		pub->popup_enum = POPUP_TELEMETRY_FLUSH;

		pub->telemetry.wait_GP = 0;

		strcpy(pub->telemetry.file_grab, FILE_TELEMETRY_GRAB);
		pub_directory_scan(pub, FILE_TELEMETRY_FILTER);
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Default")) {

		if (pub->lp->linked != 0) {

			pub->popup_enum = POPUP_RESET_DEFAULT;
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Log")) {

		if (pub->lp->linked != 0) {

			strcpy(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, FILE_TELEMETRY_LOG);
			strcpy(pub->telemetry.file_snap, pub->lbuf);

			if (link_tlm_file_open(pub->lp, pub->telemetry.file_snap) != 0) {

				pub_open_GP(pub, pub->telemetry.file_snap);
			}
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "tlm.grabfreq", "Grab into RAM frequency");
	reg_float(pub, "tlm.livefreq", "Live frequency");

	link_reg_clean_all_always(pub->lp);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_linked(pub, "tlm.reg_ID0", "Tele register ID 0");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID0");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID1", "Tele register ID 1");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID1");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID2", "Tele register ID 2");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID2");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID3", "Tele register ID 3");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID3");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID4", "Tele register ID 4");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID4");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID5", "Tele register ID 5");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID5");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID6", "Tele register ID 6");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID6");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID7", "Tele register ID 7");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID7");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID8", "Tele register ID 8");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID8");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID9", "Tele register ID 9");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID9");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	pub_popup_telemetry_flush(pub, POPUP_TELEMETRY_FLUSH, pub->popup_msg);

	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset all telemetry configuration") != 0) {

		link_command(pub->lp, "tlm_default");
		link_reg_fetch_all_shown(pub->lp);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_flash(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Program")) {

		link_command(pub->lp,	"flash_prog" "\r\n"
					"flash_info");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Wipe")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_FLASH_WIPE;
		}
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Reboot")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_SYSTEM_REBOOT;
		}
	}

	nk_spacer(ctx);

	if (pub_popup_ok_cancel(pub, POPUP_FLASH_WIPE,
				"Please confirm that you really"
				" want to WIPE flash storage") != 0) {

		link_command(pub->lp,	"flash_wipe" "\r\n"
					"flash_info");
	}

	if (pub_popup_ok_cancel(pub, POPUP_SYSTEM_REBOOT,
				"Please confirm that you really"
				" want to reboot PMC") != 0) {

		link_command(pub->lp, "rtos_reboot");
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, pub->fe_def_size_y / 3);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "INFO", NK_WINDOW_BORDER)) {

		int			N, bN, sym;

		nk_layout_row_template_begin(ctx, 0);

		for (bN = 0; bN < LINK_FLASH_MAX; ++bN) {

			if (lp->flash[0].block[bN] == 0)
				break;

			nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
		}

		nk_layout_row_template_end(ctx);

		for (N = 0; N < LINK_FLASH_MAX; ++N) {

			if (lp->flash[N].block[0] == 0)
				break;

			for (bN = 0; bN < sizeof(lp->flash[0].block); ++bN) {

				sym = lp->flash[N].block[bN];

				if (sym == 0)
					break;

				pub_drawing_brick_colored(nk, sym);
			}
		}

		nk_group_end(ctx);
	}

	if (lp->unable_warning[0] != 0) {

		strcpy(pub->popup_msg, lp->unable_warning);
		pub->popup_enum = POPUP_UNABLE_WARNING;

		lp->unable_warning[0] = 0;
	}

	pub_popup_message(pub, POPUP_UNABLE_WARNING, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_upgrade(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;

	/* TODO*/
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

			nk_group_set_scroll(ctx, "PAGE", 0, 0);

			link_reg_fetch_all_shown(pub->lp);

			pub->menu.page_pushed = pub->menu.page_current;
		}
	}

	if (pub->menu.page_current < PHOBIA_TAB_MAX - 1) {

		pub->menu.pagetab[pub->menu.page_current++] = pfunc;
	}
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

	if (nk_group_begin(ctx, "MENU", 0)) {

		pub->menu.page_current = 0;

		nk_layout_row_dynamic(ctx, 0, 1);

		menu_select_button(pub, "Serial", &page_serial);
		menu_select_button(pub, "Diagnose", &page_diagnose);
		menu_select_button(pub, "Probe", &page_probe);
		menu_select_button(pub, "HAL", &page_hal);
		menu_select_button(pub, "in Network", &page_in_network);
		menu_select_button(pub, "in STEP/DIR", &page_in_stepdir);
		menu_select_button(pub, "in PWM", &page_in_pwm);
		menu_select_button(pub, "in Knob", &page_in_knob);
		menu_select_button(pub, "Apps", &page_apps);
		menu_select_button(pub, "Thermal", &page_thermal);
		menu_select_button(pub, "Config", &page_config);
		menu_select_button(pub, "lu Forced", &page_lu_forced);
		menu_select_button(pub, "lu FLUX", &page_lu_flux);
		menu_select_button(pub, "lu HFI", &page_lu_hfi);
		menu_select_button(pub, "lu Hall", &page_lu_hall);
		menu_select_button(pub, "lu EABI", &page_lu_eabi);
		menu_select_button(pub, "lu SIN/COS", &page_lu_sincos);
		menu_select_button(pub, "Wattage", &page_wattage);
		menu_select_button(pub, "lp Current", &page_lp_current);
		menu_select_button(pub, "lp Speed", &page_lp_speed);
		menu_select_button(pub, "lp Location", &page_lp_location);
		menu_select_button(pub, "Telemetry", &page_telemetry);
		menu_select_button(pub, "Flash", &page_flash);
		menu_select_button(pub, "Upgrade", &page_upgrade);

		pub->menu.page_selected = pub->menu.page_pushed;

		nk_group_end(ctx);
	}

	if (nk_group_begin(ctx, "PAGE", 0)) {

		(void) pub->menu.pagetab[pub->menu.page_selected] (pub);

		nk_group_end(ctx);
	}
}

int main(int argc, char **argv)
{
	struct config_phobia	*fe;
	struct nk_sdl		*nk;
	struct link_pmc		*lp;
	struct public		*pub;

	setlocale(LC_NUMERIC, "C");

	fe = calloc(1, sizeof(struct config_phobia));
	nk = calloc(1, sizeof(struct nk_sdl));
	lp = calloc(1, sizeof(struct link_pmc));
	pub = calloc(1, sizeof(struct public));

	pub->fe = fe;
	pub->nk = nk;
	pub->lp = lp;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) {

		/* TODO */
	}

	if (TTF_Init() < 0) {

		/* TODO */
	}

	IMG_Init(IMG_INIT_PNG);

	config_open(pub->fe);
	pub_font_layout(pub);

	nk->window = SDL_CreateWindow("Phobia", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			pub->fe_def_size_x, pub->fe_def_size_y, SDL_WINDOW_RESIZABLE);

	if (nk->window == NULL) {

		/* TODO */
	}

	nk->window_ID = SDL_GetWindowID(nk->window);

	if (nk->window_ID == 0) {

		/* TODO */
	}

	SDL_SetWindowMinimumSize(nk->window, pub->fe_def_size_x, pub->fe_def_size_y);
	SDL_StopTextInput();

	nk->fb = SDL_GetWindowSurface(nk->window);

	if (nk->fb == NULL) {

		/* TODO */
	}

	nk->surface = SDL_CreateRGBSurfaceWithFormat(0, nk->fb->w,
			nk->fb->h, 32, SDL_PIXELFORMAT_XRGB8888);

	if (nk->surface == NULL) {

		/* TODO */
	}

	nk_init_default(&nk->ctx, &nk->font);
	nk_sdl_style_custom(nk, pub->fe_padding);

	while (nk->onquit == 0) {

		SDL_Event		ev;

		nk->clock = SDL_GetTicks();

		nk_input_begin(&nk->ctx);

		while (SDL_PollEvent(&ev) != 0) {

			if (		ev.window.windowID == nk->window_ID
					|| ev.type == SDL_QUIT) {

				nk_sdl_input_event(nk, &ev);
			}
			else if (	pub->gp != NULL
					&& ev.window.windowID == pub->gp_ID) {

				gp_TakeEvent(pub->gp, &ev);
			}

			nk->active = 1;
		}

		nk_input_end(&nk->ctx);

		if (link_fetch(pub->lp, nk->clock) != 0) {

			nk->active = 1;
		}

		if (nk->active != 0) {

			nk->idled = 0;
		}
		else {
			nk->idled += 1;
			nk->active = (nk->idled < 100) ? 1 : 0;
		}

		if (nk->active != 0) {

			struct nk_rect		bounds = nk_rect(0, 0, nk->surface->w,
									nk->surface->h);

			if (lp->hwinfo[0] != 0) {

				nk->ctx.style.window.header.active =
					nk_style_item_color(nk->table[NK_COLOR_ENABLED]);

				sprintf(pub->lbuf, " %.77s", lp->hwinfo);

				if (lp->network[0] != 0) {

					sprintf(pub->lbuf + strlen(pub->lbuf),
							" [%.32s]", lp->network);
				}
			}
			else {
				nk->ctx.style.window.header.active =
					nk_style_item_color(nk->table[NK_COLOR_HEADER]);

				sprintf(pub->lbuf, " Not linked to PMC");
			}

			if (nk_begin(&nk->ctx, pub->lbuf, bounds, NK_WINDOW_TITLE)) {

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

		if (		pub->gp != NULL
				&& gp_IsQuit(pub->gp) == 0) {

			if (gp_Draw(pub->gp) == 0) {

				SDL_Delay(10);
			}
		}
		else {
			if (pub->gp != NULL) {

				if (lp->grab_N != 0) {

					link_grab_file_close(lp);
				}

				if (lp->tlm_N != 0) {

					link_tlm_file_close(lp);
				}

				link_command(lp, "\r\n");

				gp_Clean(pub->gp);

				pub->gp = NULL;
			}

			SDL_Delay(10);
		}
	}

	if (pub->gp != NULL) {

		gp_Clean(pub->gp);
	}

	link_close(lp);
	config_write(pub->fe);

	free(nk);
	free(lp);
	free(pub);

	SDL_Quit();

	return 0;
}

