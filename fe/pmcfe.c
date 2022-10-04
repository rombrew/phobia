#include <stdlib.h>
#include <string.h>
#include <locale.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "config.h"
#include "dirent.h"
#include "link.h"
#include "serial.h"
#include "nksdl.h"

#undef main

#define PMCFE_FILE_MAX				200
#define PMCFE_TAB_MAX				40

SDL_RWops *TTF_RW_droid_sans_normal();

enum {
	POPUP_NONE			= 0,
	POPUP_RESET_DEFAULT,
	POPUP_UNABLE_WARNING,
	POPUP_FLASH_WIPE,
	POPUP_SYSTEM_REBOOT,
	POPUP_TELEMETRY_FLUSH,
};

struct public {

	struct config_pmcfe	*fe;
	struct nk_sdl		*nk;
	struct link_pmc		*lp;

	int			fe_def_size_x;
	int			fe_def_size_y;
	int			fe_font_h;
	int			fe_padding;
	int			fe_base;

	int			combo_count;

	struct {

		int			page_current;
		int			page_selected;
		int			page_pushed;

		void			(* pagetab [PMCFE_TAB_MAX]) (struct public *);
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

		struct dirent_stat	sb;

		struct {

			char			name[DIRENT_PATH_MAX];
			char			time[24];
			char			size[24];
		}
		file[PMCFE_FILE_MAX];

		int			selected;
	}
	scan;

	struct {

		int			autoupdate;
		int			wait_GP;

		char			file_snap[PMCFE_PATH_MAX];
		char			file_grab[PMCFE_NAME_MAX];
	}
	telemetry;

	char			popup_msg[LINK_MESSAGE_MAX];
	int			popup_enum;

	char			lbuf[PMCFE_PATH_MAX];
};

static void
pub_font_layout(struct public *pub)
{
	struct config_pmcfe		*fe = pub->fe;
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

			if (		   strcmp(reg->sym, "pm.fault_current_halt") == 0
					|| strcmp(reg->sym, "pm.probe_speed_hold") == 0
					|| strcmp(reg->sym, "pm.zone_threshold_BASE") == 0
					|| strcmp(reg->sym, "pm.i_maximal") == 0
					|| strcmp(reg->sym, "pm.i_gain_P") == 0
					|| strcmp(reg->sym, "pm.s_gain_P") == 0) {

				if (nk_contextual_item_label(ctx, "Automatic request",
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
				pub->fe->fuzzy, sizeof(pub->fe->fuzzy),
				nk_filter_default);

		pub->combo_count = 0;

		nk_layout_row_dynamic(ctx, (float) item_height, 1);

		for (reg_ID = 0; reg_ID < LINK_REGS_MAX; ++reg_ID) {

			if (lp->reg[reg_ID].sym[0] == 0)
				break;

			if (strstr(lp->reg[reg_ID].sym, pub->fe->fuzzy) != NULL) {

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
pub_get_popup_bounds_tiny(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			win, popup;

	win = nk_window_get_content_region(ctx);

	popup.w = win.w * 0.75f;
	popup.h = win.h * 0.55f;

	popup.x = win.w / 2.f - popup.w / 2.f;
	popup.y = win.h * 0.3f;

	return popup;
}

static struct nk_rect
pub_get_popup_bounds_full(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			win, popup;

	win = nk_window_get_content_region(ctx);

	popup.w = win.w * 0.95f;
	popup.h = win.h * 0.95f;

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

	return rc;
}

static void
pub_filename_generate(struct public *pub, char *filebuf, const char *fmt)
{
	FILE		*fd_try;
	int		i;

	for (i = 0; i < 99; ++i) {

		sprintf(filebuf, fmt, i);

		strcpy(pub->lbuf, pub->fe->storage);
		strcat(pub->lbuf, DIRSEP);
		strcat(pub->lbuf, filebuf);

		fd_try = fopen_from_UTF8(pub->lbuf, "r");

		if (fd_try == NULL) {

			break;
		}
		else {
			fclose(fd_try);
		}
	}
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

				if (N >= PMCFE_FILE_MAX - 1)
					break;
			}
		}
	}

	dirent_close(&pub->scan.sb);

	pub->scan.file[N].name[0] = 0;
	pub->scan.selected = - 1;
}

static void
pub_run_GP(struct public *pub, const char *file)
{
	FILE			*fd;

	if (strcmp(pub->fe->gpcmd, "gp") == 0) {

		const char	*lconfgp = ".feplot.gp";

		strcpy(pub->lbuf, pub->fe->storage);
		strcat(pub->lbuf, DIRSEP);
		strcat(pub->lbuf, lconfgp);

		fd = fopen_from_UTF8(pub->lbuf, "w");

		if (fd != NULL) {

			fprintf(fd,	"chunk 10\n"
					"load 0 0 text \"%s\"\n"
					"mkpages -1\n", file);
			/*
					"group 0 -1\n"
					"deflabel 0 \"Time (s)\"\n"
					"defscale 0 %.7E 0\n"
			 */

			fclose(fd);

			strcpy(pub->lbuf, pub->fe->gpcmd);
			strcat(pub->lbuf, " ");
			strcat(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, lconfgp);

			system_async(pub->lbuf);
		}
	}
	else {
		strcpy(pub->lbuf, pub->fe->gpcmd);
		strcat(pub->lbuf, " ");
		strcat(pub->lbuf, file);

		system_async(pub->lbuf);
	}
}

static void
pub_popup_telemetry_flush(struct public *pub, int popup, const char *title)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_rect			bounds;
	struct nk_style_button		disabled;

	int				fm_height;
	int				N, sel, newsel;

	if (pub->popup_enum != popup)
		return ;

	bounds = pub_get_popup_bounds_full(pub);

	if (nk_popup_begin(ctx, NK_POPUP_STATIC, title,
				NK_WINDOW_CLOSABLE
				| NK_WINDOW_NO_SCROLLBAR, bounds)) {

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		nk_layout_row_template_begin(ctx, 0);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 5);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_variable(ctx, 1);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 5);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		nk_spacer(ctx);

		if (lp->grab_status == GRAB_FINISHED) {

			pub_filename_generate(pub, pub->telemetry.file_grab, FILE_GRAB_FORMAT);
			pub_directory_scan(pub, FILE_GRAB_SUFFIX);

			lp->grab_status = GRAB_DEFUNCT;
		}

		sprintf(pub->lbuf, "# %i", lp->grab_fetched_N);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD,
				pub->telemetry.file_grab,
				sizeof(pub->telemetry.file_grab),
				nk_filter_default);

		if (		pub->scan.selected >= 0
				&& pub->scan.selected < PMCFE_FILE_MAX) {

			N = pub->scan.selected;

			if (		strcmp(pub->scan.file[N].name,
						pub->telemetry.file_grab) != 0) {

				pub->scan.selected = -1;
			}
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "New")) {

			pub_filename_generate(pub, pub->telemetry.file_grab, FILE_GRAB_FORMAT);
			pub_directory_scan(pub, FILE_GRAB_SUFFIX);
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		fm_height = ctx->current->bounds.h - ctx->current->layout->row.height * 7;

		nk_layout_row_template_begin(ctx, fm_height);
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

			for (N = 0; N < PMCFE_FILE_MAX; ++N) {

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

		if (nk_button_label(ctx, "Flush GP")) {

			strcpy(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, pub->telemetry.file_grab);
			strcpy(pub->telemetry.file_snap, pub->lbuf);

			if (link_grab_file(lp, pub->telemetry.file_snap) != 0) {

				if (link_command(pub->lp, "tlm_flush_sync") != 0) {

					pub->telemetry.wait_GP = 1;
				}
			}
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Live GP")) {

			strcpy(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, pub->telemetry.file_grab);
			strcpy(pub->telemetry.file_snap, pub->lbuf);

			if (link_grab_file(lp, pub->telemetry.file_snap) != 0) {

				if (link_command(pub->lp, "tlm_live_sync") != 0) {

					pub->telemetry.wait_GP = 1;
				}
			}
		}

		nk_spacer(ctx);

		if (		pub->scan.selected >= 0
				&& pub->scan.selected < PMCFE_FILE_MAX) {

			if (nk_button_label(ctx, "Plot GP")) {

				strcpy(pub->lbuf, pub->fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, pub->telemetry.file_grab);
				strcpy(pub->telemetry.file_snap, pub->lbuf);

				pub_run_GP(pub, pub->telemetry.file_snap);
			}

			nk_spacer(ctx);

			if (nk_button_label(ctx, "Remove")) {

				strcpy(pub->lbuf, pub->fe->storage);
				strcat(pub->lbuf, DIRSEP);
				strcat(pub->lbuf, pub->telemetry.file_grab);

				file_remove(pub->lbuf);

				pub_directory_scan(pub, FILE_GRAB_SUFFIX);
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
				&& lp->grab_fetched_N >= 10) {

			pub->telemetry.wait_GP = 0;

			pub_run_GP(pub, pub->telemetry.file_snap);
		}

		nk_popup_end(ctx);
	}
	else {
		pub->popup_enum = 0;
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

		pub_name_label(pub, name, reg);

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

	int				pc;

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

		if (reg->started != 0) {

			pc = (int) (1000.f * (reg->fval - reg->fmin)
					/ (reg->fmax - reg->fmin));
		}
		else {
			pc = 0;
		}

		nk_prog(ctx, pc, 1000, nk_false);
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

		reg->update = (pub->telemetry.autoupdate != 0) ? 200 : 0;
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
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	struct nk_style_button		orange;

	const char			*ls_baudrates[] = {

		"1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"
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

	if (pub->lp->linked == 0) {

		if (pub->serial.started == 0) {

			serial_enumerate(&pub->serial.list);

			pub->serial.started = 1;
			pub->serial.baudrate = 6;
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
		pub->serial.baudrate = nk_combo(ctx, ls_baudrates, 8,
				pub->serial.baudrate, pub->fe_font_h
				+ pub->fe_padding, nk_vec2(pub->fe_base * 13, 400));

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Connect")) {

			const char		*portname;
			int			baudrate = 0;

			portname = pub->serial.list.name[pub->serial.selected];
			lk_stoi(&baudrate, ls_baudrates[pub->serial.baudrate]);

			link_open(pub->lp, portname, baudrate);

			strcpy(pub->lbuf, pub->fe->storage);
			strcat(pub->lbuf, DIRSEP);
			strcat(pub->lbuf, FILE_LINK_LOG);

			link_log_file(pub->lp, pub->lbuf);
		}
	}
	else {
		nk_label(ctx, "Serial port", NK_TEXT_LEFT);
		nk_label(ctx, pub->lp->devname, NK_TEXT_LEFT);

		nk_spacer(ctx);

		sprintf(pub->lbuf, "# %i", pub->lp->fetched_N);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		nk_label(ctx, "Baudrate", NK_TEXT_LEFT);
		sprintf(pub->lbuf, "%i", pub->lp->baudrate);
		nk_label(ctx, pub->lbuf, NK_TEXT_LEFT);

		nk_spacer(ctx);

		if (nk_button_label_styled(ctx, &orange, "Drop")) {

			link_close(pub->lp);
		}
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 22);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_label_colored(ctx, "PMCFE configuration file", NK_TEXT_LEFT,
			nk->table[NK_COLOR_CONFIG]);

	nk_edit_string_zero_terminated(ctx, NK_EDIT_SELECTABLE,
			pub->fe->rcfile, sizeof(pub->fe->rcfile),
			nk_filter_default);

	nk_spacer(ctx);

	nk_label(ctx, "Window layout", NK_TEXT_LEFT);

	select = nk_combo(ctx, ls_windowsize, 3, pub->fe->windowsize, pub->fe_font_h
			+ pub->fe_padding, nk_vec2(pub->fe_base * 22, 400));

	if (select != pub->fe->windowsize) {

		pub->fe->windowsize = select;

		config_write(pub->fe);
		pub_font_layout(pub);
	}

	nk_spacer(ctx);

	nk_label(ctx, "Storage PATH", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, pub->fe->storage,
			sizeof(pub->fe->storage), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_label(ctx, "Linked register pattern", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, pub->fe->fuzzy,
			sizeof(pub->fe->fuzzy), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_label(ctx, "GP tool", NK_TEXT_LEFT);

	rc = nk_edit_string_zero_terminated(ctx, NK_EDIT_FIELD
			| NK_EDIT_SIG_ENTER, pub->fe->gpcmd,
			sizeof(pub->fe->gpcmd), nk_filter_default);

	if (rc & (NK_EDIT_DEACTIVATED | NK_EDIT_COMMITED)) {

		config_write(pub->fe);
	}

	nk_spacer(ctx);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Default")) {

		pub->popup_enum = POPUP_RESET_DEFAULT;
	}

	nk_spacer(ctx);


	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset PMCFE configuration") != 0) {

		config_default(pub->fe);
		config_write(pub->fe);

		pub_font_layout(pub);
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
	reg_float(pub, "pm.const_fb_U", "DC link voltage");
	reg_float(pub, "pm.ad_IA_0", "A sensor drift");
	reg_float(pub, "pm.ad_IB_0", "B sensor drift");
	reg_float(pub, "pm.ad_IC_0", "C sensor drift");
	reg_text_large(pub, "pm.self_BST", "Bootstrap retention time");
	reg_text_large(pub, "pm.self_BM", "Self test result");
	reg_text_large(pub, "pm.self_RMSi", "Current sensor RMS");
	reg_text_large(pub, "pm.self_RMSu", "Voltage sensor RMS");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.ad_IA_1", "A sensor scale");
	reg_float(pub, "pm.ad_IB_1", "B sensor scale");
	reg_float(pub, "pm.ad_IC_1", "C sensor scale");
	reg_float(pub, "pm.ad_UA_0", "A voltage offset");
	reg_float(pub, "pm.ad_UA_1", "A voltage scale");
	reg_float(pub, "pm.ad_UB_0", "B voltage offset");
	reg_float(pub, "pm.ad_UB_1", "B voltage scale");
	reg_float(pub, "pm.ad_UC_0", "C voltage offset");
	reg_float(pub, "pm.ad_UC_1", "C voltage scale");
	reg_enum_toggle(pub, "pm.tvm_INUSE", "DT compensation");
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
	reg_float(pub, "pm.fb_HS", "HALL sensors feedback");
	reg_float(pub, "pm.fb_EP", "ABI encoder feedback");
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

			reg = link_reg_lookup(pub->lp, "pm.lu_load_torque");
			if (reg != NULL) { reg->update = rate; }
		}

		reg_float(pub, "pm.lu_iD", "LU current D");
		reg_float(pub, "pm.lu_iQ", "LU current Q");
		reg_float_um(pub, "pm.lu_wS", "LU speed estimate", 1);
		reg_float(pub, "pm.lu_load_torque", "LU torque estimate");

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
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
		nk_layout_row_template_push_static(ctx, pub->fe_base);
		nk_layout_row_template_end(ctx);

		if (nk_button_label(ctx, "Probe E")) {

			link_command(pub->lp, "pm_probe_const_E");
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Probe J")) {

			link_command(pub->lp, "pm_probe_const_J");
		}

		nk_spacer(ctx);

		if (nk_button_label(ctx, "Probe NT")) {

			link_command(pub->lp, "pm_probe_noise_threshold");
		}

		nk_spacer(ctx);

		nk_layout_row_dynamic(ctx, 0, 1);
		nk_spacer(ctx);

		reg_float(pub, "pm.zone_threshold_NOISE", "NOISE threshold");
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
	reg_float(pub, "hal.ADC_knob_ratio", "Knob voltage divider ratio");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "hal.DPS_mode", "DPS operation mode");
	reg_enum_combo(pub, "hal.PPM_mode", "PPM operation mode");
	reg_float(pub, "hal.PPM_timebase", "PPM time base");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "hal.DRV.part", "DRV part");
	reg_float(pub, "hal.DRV.auto_RESET", "DRV automatic reset");
	reg_float(pub, "hal.DRV.status_raw", "DRV status raw");
	reg_float(pub, "hal.DRV.gate_current", "DRV gate current");
	reg_float(pub, "hal.DRV.ocp_level", "DRV OCP level");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "hal.OPT", "OPT bitmask raw");

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
page_in_PWM(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "hal.PPM_caught");

	if (reg != NULL) {

		int		rate;

		reg->update = 200;
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
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range_1");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.ppm_control_range_2");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.idle_TIME_s", "IDLE timeout");

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
	reg_enum_toggle(pub, "ap.knob_STARTUP", "Startup on ANG in range");
	reg_float(pub, "ap.knob_in_ANG_0", "ANG range 0");
	reg_float(pub, "ap.knob_in_ANG_1", "ANG range 1");
	reg_float(pub, "ap.knob_in_ANG_2", "ANG range 2");
	reg_float(pub, "ap.knob_in_BRK_0", "BRK range 0");
	reg_float(pub, "ap.knob_in_BRK_1", "BRK range 1");
	reg_float(pub, "ap.knob_in_lost_0", "Lost range 0");
	reg_float(pub, "ap.knob_in_lost_1", "Lost range 1");
	reg_float(pub, "ap.knob_control_ANG_0", "Control range 0");
	reg_float(pub, "ap.knob_control_ANG_1", "Control range 1");
	reg_float(pub, "ap.knob_control_ANG_2", "Control range 2");
	reg_float(pub, "ap.knob_control_BRK", "BRK control");

	reg = link_reg_lookup(pub->lp, "ap.knob_reg_ID");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG_0");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG_1");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_ANG_2");
		if (reg != NULL) { reg->onefetch = 1; }

		reg = link_reg_lookup(pub->lp, "ap.knob_control_BRK");
		if (reg != NULL) { reg->onefetch = 1; }
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.idle_TIME_s", "IDLE timeout");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
page_apps(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;

	reg_enum_combo(pub, "ap.autorun_APP_0", "Autorun APP 0");
	reg_enum_combo(pub, "ap.autorun_APP_1", "Autorun APP 1");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "ap.hx711_kg", "HX711 measurement");
	reg_float(pub, "ap.hx711_scale_0", "HX711 load offset");
	reg_float(pub, "ap.hx711_scale_1", "HX711 load scale");

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

	reg_enum_combo(pub, "ap.ntc_PCB.type", "NTC type (PCB)");
	reg_float(pub, "ap.ntc_PCB.balance", "NTC balance (PCB)");
	reg_float(pub, "ap.ntc_PCB.ntc_0", "NTC resistance at Ta");
	reg_float(pub, "ap.ntc_PCB.ta_0", "NTC Ta (PCB)");
	reg_float(pub, "ap.ntc_PCB.betta", "NTC betta (PCB)");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "ap.ntc_EXT.type", "NTC type (EXT)");
	reg_float(pub, "ap.ntc_EXT.balance", "NTC balance (EXT)");
	reg_float(pub, "ap.ntc_EXT.ntc_0", "NTC resistance at Ta");
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
	reg_float(pub, "ap.temp_INT", "Temperature MCU");

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

	reg_float(pub, "pm.dc_resolution", "PWM resolution");
	reg_float(pub, "pm.dc_minimal", "Minimal pulse");
	reg_float(pub, "pm.dc_clearance", "Clearance before ADC sample");
	reg_float(pub, "pm.dc_skip", "Skip after ADC sample");
	reg_float(pub, "pm.dc_bootstrap", "Bootstrap retention time");
	reg_float(pub, "pm.dc_clamped", "Bootstrap recharge time");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_combo(pub, "pm.config_NOP", "Number of motor phases");
	reg_enum_combo(pub, "pm.config_IFB", "Current measurement scheme");
	reg_enum_toggle(pub, "pm.config_TVM", "Terminal voltage measurement");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_enum_toggle(pub, "pm.config_VSI_CIRCULAR", "Circular SVPWM clamping");
	reg_enum_toggle(pub, "pm.config_VSI_PRECISE", "Precise SVPWM (in middle)");
	reg_enum_toggle(pub, "pm.config_LU_FORCED", "Forced control");
	reg_enum_combo(pub, "pm.config_LU_ESTIMATE_FLUX", "Estimate FLUX");
	reg_enum_toggle(pub, "pm.config_LU_ESTIMATE_HFI", "Estimate HFI");
	reg_enum_combo(pub, "pm.config_LU_SENSOR", "Position SENSOR");
	reg_enum_combo(pub, "pm.config_LU_LOCATION", "LOCATION source");
	reg_enum_combo(pub, "pm.config_LU_DRIVE", "Drive loop");
	reg_enum_toggle(pub, "pm.config_RELUCTANCE", "Reluctance motor");
	reg_enum_toggle(pub, "pm.config_WEAKENING", "Flux weakening");
	reg_enum_toggle(pub, "pm.config_HOLDING_BRAKE", "Holding brake (CC)");
	reg_enum_toggle(pub, "pm.config_SPEED_LIMITED", "Speed limit (CC)");
	reg_enum_toggle(pub, "pm.config_IMPEDANCE_MAJOR", "Major axes reverse");
	reg_enum_toggle(pub, "pm.config_ABI_ABSOLUTE", "ABI sensor absolute");
	reg_enum_combo(pub, "pm.config_SINCOS_FRONTEND", "SINCOS frontend");
	reg_enum_toggle(pub, "pm.config_MILEAGE_INFO", "Mileage info");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.tm_transient_slow", "Transient time (slow)");
	reg_float(pub, "pm.tm_transient_fast", "Transient time (fast)");
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
	reg_float(pub, "pm.probe_freq_sine_hz", "Probe sine frequency");
	reg_float_um(pub, "pm.probe_speed_hold", "Probe speed", 0);
	reg_float_um(pub, "pm.probe_speed_detached", "Probe speed (detached)", 0);
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

	reg_float(pub, "pm.lu_transient", "LU transient rate");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 8);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Default")) {

		if (lp->linked != 0) {

			pub->popup_enum = POPUP_RESET_DEFAULT;
		}
	}

	if (pub_popup_ok_cancel(pub, POPUP_RESET_DEFAULT,
				"Please confirm that you really"
				" want to reset PMC configuration") != 0) {

		link_command(pub->lp, "pm_default_config");
		link_reg_fetch_all_shown(pub->lp);
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Save TXT")) {

		/* TODO */
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Load TXT")) {

		/* TODO */
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
	reg_float(pub, "pm.forced_slew_rate", "Current slew rate");
	reg_float(pub, "pm.forced_maximal_DC", "Maximal DC usage");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
}

static void
page_lu_FLUX(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate;

		reg->update = 1000;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "pm.flux_E");
		if (reg != NULL) {  reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.flux_wS");
		if (reg != NULL) { reg += reg->um_sel; reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.config_LU_ESTIMATE_FLUX");

		if (		reg != NULL
				&& reg->lval == 2) {

			reg = link_reg_lookup(pub->lp, "pm.kalman_Z");
			if (reg != NULL) { reg->update = rate; }
		}

		reg = link_reg_lookup(pub->lp, "pm.flux_ZONE");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);
	reg_float(pub, "pm.flux_E", "Flux linkage");
	reg_float_um(pub, "pm.flux_wS", "Speed estimate", 0);
	reg_float(pub, "pm.kalman_Z", "Q relaxation drift");
	reg_enum_errno(pub, "pm.flux_ZONE", "Speed ZONE", 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.detach_level_U", "Detached voltage level");
	reg_float(pub, "pm.detach_trip_AD", "Detached trip gain");
	reg_float(pub, "pm.detach_gain_SF", "Detached speed loop gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.flux_trip_AD", "Speed trip gain");
	reg_float(pub, "pm.flux_gain_IN", "Initial gain");
	reg_float(pub, "pm.flux_gain_LO", "Low flux gain");
	reg_float(pub, "pm.flux_gain_HI", "High flux gain");
	reg_float(pub, "pm.flux_gain_SF", "Speed loop gain");
	reg_float(pub, "pm.flux_gain_IF", "Torque gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.kalman_gain_Q_0", "Kalman Q gain 0");
	reg_float(pub, "pm.kalman_gain_Q_1", "Kalman Q gain 1");
	reg_float(pub, "pm.kalman_gain_Q_2", "Kalman Q gain 2");
	reg_float(pub, "pm.kalman_gain_Q_3", "Kalman Q gain 3");
	reg_float(pub, "pm.kalman_gain_Q_4", "Kalman Q gain 4");
	reg_float(pub, "pm.kalman_gain_R", "Kalman R gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float_um(pub, "pm.zone_threshold_NOISE", "NOISE threshold", 0);
	reg_float_um(pub, "pm.zone_threshold_BASE", "BASE threshold", 0);
	reg_float(pub, "pm.zone_gain_HY", "Hysteresis gain");
	reg_float(pub, "pm.zone_gain_LP_S", "Speed LPF gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
}

static void
pub_drawing_HFI(struct public *pub, float Fa, int Zp)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct nk_command_buffer	*canvas = nk_window_get_canvas(ctx);
	struct nk_rect			space;
	enum nk_widget_layout_states	state;

	float				tfm[4], arrow[12], pbuf[12];
	float				radius, thickness;
	int				tooth;

	state = nk_widget(&space, ctx);
	if (!state) return ;

	radius = space.w / 2.f;
	thickness = radius / 20.f;

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

	if (Zp > 1) {

		arrow[0] = .7f * radius;
		arrow[1] = - .5f * thickness;
		arrow[2] = .7f * radius;
		arrow[3] = .5f * thickness;
		arrow[4] = radius - 1.5f * thickness;
		arrow[5] = .5f * thickness;
		arrow[6] = radius - 1.5f * thickness;
		arrow[7] = 0.f;
		arrow[8] = radius - 1.5f * thickness;
		arrow[9] = - .5f * thickness;
	}
	else {
		Zp = 0;
	}

	for (tooth = 0; tooth < Zp; ++tooth) {

		float		Za = (float) tooth * (float) (2. * M_PI) / (float) Zp;

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
		pbuf[8] = tfm[0] + (tfm[2] * arrow[8] - tfm[3] * arrow[9]);
		pbuf[9] = tfm[1] + (tfm[3] * arrow[8] + tfm[2] * arrow[9]);

		nk_fill_polygon(canvas, pbuf, 5, nk->table[NK_COLOR_DRAWING_PEN]);
	}

	if (isfinite(Fa) != 0) {

		tfm[2] = cosf(Fa);
		tfm[3] = - sinf(Fa);

		arrow[0] = - .5f * thickness;
		arrow[1] = - .5f * thickness;
		arrow[2] = - .5f * thickness;
		arrow[3] = .5f * thickness;
		arrow[4] = radius - 1.5f * thickness;
		arrow[5] = .5f * thickness;
		arrow[6] = radius - 1.2f * thickness;
		arrow[7] = 0.f;
		arrow[8] = radius - 1.5f * thickness;
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
}

static void
page_lu_HFI(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	int				dw_HFI = pub->fe_def_size_x / 3;

	reg_float(pub, "pm.hfi_inject_sine", "Injected current");
	reg_float_um(pub, "pm.hfi_maximal", "Maximal speed", 0);
	reg_float(pub, "pm.hfi_INJS", "Frequency ratio");
	reg_float(pub, "pm.hfi_SKIP", "Skipped cycles");
	reg_float(pub, "pm.hfi_ESTI", "Estimate cycles");
	reg_float(pub, "pm.hfi_POLA", "Flux polarity");
	reg_float(pub, "pm.hfi_gain_SF", "Speed loop gain");
	reg_float(pub, "pm.hfi_gain_IF", "Torque gain");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate, fast;

		reg->update = 1000;

		rate = (reg->lval != 0) ? 200 : 0;
		fast = (reg->lval != 0) ? 20  : 0;

		reg = link_reg_lookup(pub->lp, "pm.lu_location");
		if (reg != NULL) { reg += reg->um_sel; reg->update = fast; }

		reg = link_reg_lookup(pub->lp, "pm.hfi_im_L1");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.hfi_im_L2");
		if (reg != NULL) { reg->update = rate; }

		reg = link_reg_lookup(pub->lp, "pm.hfi_im_R");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_enum_errno(pub, "pm.lu_MODE", "LU operation mode", 0);
	reg_float_um(pub, "pm.lu_location", "LU absolute location", 1);
	reg_float(pub, "pm.hfi_im_L1", "HF inductance L1");
	reg_float(pub, "pm.hfi_im_L2", "HF inductance L2");
	reg_float(pub, "pm.hfi_im_R", "HF active impedance");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, dw_HFI + 20);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_static(ctx, dw_HFI + 20);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "HFI", NK_WINDOW_BORDER)) {

		nk_layout_row_static(ctx, dw_HFI, dw_HFI, 1);

		reg = link_reg_lookup(pub->lp, "pm.lu_location");

		if (reg != NULL) {

			if (reg->um_sel == 0) {

				reg += reg->um_sel;

				pub_drawing_HFI(pub, reg->fval, 0);
			}
			else if (reg->um_sel == 1) {

				float		Fa;
				int		Zp = 0;

				reg += reg->um_sel;
				Fa = reg->fval * (float) (M_PI / 180.);

				reg = link_reg_lookup(pub->lp, "pm.const_Zp");
				if (reg != NULL) { Zp = reg->lval; }

				pub_drawing_HFI(pub, Fa, Zp);
			}
			else if (reg->um_sel == 2) {

				reg += reg->um_sel;

				pub_drawing_HFI(pub, reg->fval, 0);
			}
		}

		nk_group_end(ctx);
	}

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
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

	reg = link_reg_lookup(pub->lp, "pm.lu_MODE");

	if (reg != NULL) {

		int		rate;

		reg->update = 1000;
		reg->shown = pub->lp->clock;

		rate = (reg->lval != 0) ? 100 : 0;

		reg = link_reg_lookup(pub->lp, "pm.watt_lpf_wP");
		if (reg != NULL) { reg->update = rate; }
	}

	reg_float(pub, "pm.watt_lpf_wP", "Wattage at now");

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

	reg_float_um(pub, "pm.i_setpoint_current", "Current setpoint", 0);
	reg_float(pub, "pm.i_maximal", "Maximal forward current");
	reg_float(pub, "pm.i_reverse", "Maximal reverse current");
	reg_float(pub, "pm.i_slew_rate", "Slew rate");
	reg_float(pub, "pm.i_tol_Z", "Dead Zone");
	reg_float(pub, "pm.i_gain_P", "Proportional gain P");
	reg_float(pub, "pm.i_gain_I", "Integral gain I");

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
page_speed(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct nk_context		*ctx = &nk->ctx;
	struct link_reg			*reg;

	reg_float_um(pub, "pm.s_setpoint_speed", "Speed setpoint", 0);
	reg_float_um(pub, "pm.s_maximal", "Maximal forward speed", 0);
	reg_float_um(pub, "pm.s_reverse", "Maximal reverse speed", 0);
	reg_float_um(pub, "pm.s_accel", "Maximal acceleration", 0);
	reg_float(pub, "pm.s_linspan", "Regulation span (CC)");
	reg_float(pub, "pm.s_tol_Z", "Dead Zone");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_float(pub, "pm.s_gain_P", "Proportional gain P");
	reg_float(pub, "pm.lu_gain_TQ", "Torque estimate gain");

	reg = link_reg_lookup(pub->lp, "pm.s_gain_P");

	if (		reg != NULL
			&& reg->fetched == pub->lp->clock) {

		reg = link_reg_lookup(pub->lp, "pm.lu_gain_TQ");
		if (reg != NULL) { reg->onefetch = 1; }
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

	reg_float(pub, "pm.lu_total_revol", "Total revolutions");
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

	reg_float(pub, "tlm.freq_grab_hz", "Single grab frequency");
	reg_float(pub, "tlm.freq_live_hz", "Live data frequency");

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	nk_layout_row_template_begin(ctx, 0);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 7);
	nk_layout_row_template_push_static(ctx, pub->fe_base);
	nk_layout_row_template_end(ctx);

	if (nk_button_label(ctx, "Grab")) {

		link_command(pub->lp, "tlm_grab");
	}

	nk_spacer(ctx);

	if (nk_button_label(ctx, "Flush")) {

		strcpy(pub->popup_msg, "GP telemetry flush tool");
		pub->popup_enum = POPUP_TELEMETRY_FLUSH;

		pub->telemetry.wait_GP = 0;

		pub_filename_generate(pub, pub->telemetry.file_grab, FILE_GRAB_FORMAT);
		pub_directory_scan(pub, FILE_GRAB_SUFFIX);
	}

	nk_spacer(ctx);

	pub->telemetry.autoupdate = (pub->popup_enum != POPUP_NONE) ? 0 : 1;

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	reg_linked(pub, "tlm.reg_ID_0", "Tele register ID 0");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_0");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_1", "Tele register ID 1");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_1");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_2", "Tele register ID 2");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_2");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_3", "Tele register ID 3");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_3");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_4", "Tele register ID 4");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_4");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_5", "Tele register ID 5");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_5");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_6", "Tele register ID 6");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_6");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_7", "Tele register ID 7");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_7");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_8", "Tele register ID 8");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_8");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	reg_linked(pub, "tlm.reg_ID_9", "Tele register ID 9");

	reg = link_reg_lookup(pub->lp, "tlm.reg_ID_9");
	reg_float_prog(pub, (reg != NULL) ? reg->lval : 0);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);

	pub_popup_telemetry_flush(pub, POPUP_TELEMETRY_FLUSH, pub->popup_msg);

	nk_layout_row_dynamic(ctx, 0, 1);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
	nk_spacer(ctx);
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
page_flash(struct public *pub)
{
	struct nk_sdl			*nk = pub->nk;
	struct link_pmc			*lp = pub->lp;
	struct nk_context		*ctx = &nk->ctx;

	int				page, block, page_len, sym;

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

	page_len = strlen(lp->flash_info_map[0]);

	nk_layout_row_template_begin(ctx, pub->fe_def_size_x / 3);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_push_variable(ctx, 1);
	nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
	nk_layout_row_template_end(ctx);

	nk_spacer(ctx);

	if (nk_group_begin(ctx, "INFO", NK_WINDOW_BORDER)) {

		nk_layout_row_template_begin(ctx, 0);

		for (block = 0; block < page_len; ++block) {

			nk_layout_row_template_push_static(ctx, pub->fe_base * 2);
		}

		nk_layout_row_template_end(ctx);

		for (page = 0; page < 8; ++page) {

			if (strlen(lp->flash_info_map[page]) == 0)
				break;

			for (block = 0; block < page_len; ++block) {

				sym = lp->flash_info_map[page][block];

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

	if (pub->menu.page_current < PMCFE_TAB_MAX - 1) {

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
		menu_select_button(pub, "HAL", &page_HAL);
		menu_select_button(pub, "in Network", &page_in_network);
		menu_select_button(pub, "in STEP/DIR", &page_in_STEPDIR);
		menu_select_button(pub, "in PWM", &page_in_PWM);
		menu_select_button(pub, "in Knob", &page_in_knob);
		menu_select_button(pub, "Apps", &page_apps);
		menu_select_button(pub, "Thermal", &page_thermal);
		menu_select_button(pub, "Config", &page_config);
		menu_select_button(pub, "lu Forced", &page_lu_forced);
		menu_select_button(pub, "lu FLUX", &page_lu_FLUX);
		menu_select_button(pub, "lu HFI", &page_lu_HFI);
		menu_select_button(pub, "lu HALL", &page_lu_HALL);
		menu_select_button(pub, "lu ABI", &page_lu_ABI);
		menu_select_button(pub, "lu SIN/COS", &page_lu_SINCOS);
		menu_select_button(pub, "Wattage", &page_wattage);
		menu_select_button(pub, "lp Current", &page_current);
		menu_select_button(pub, "lp Speed", &page_speed);
		menu_select_button(pub, "lp Servo", &page_servo);
		menu_select_button(pub, "Mileage", &page_mileage);
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
	struct config_pmcfe	*fe;
	struct nk_sdl		*nk;
	struct link_pmc		*lp;
	struct public		*pub;

	setlocale(LC_NUMERIC, "C");

	fe = calloc(1, sizeof(struct config_pmcfe));
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

	config_open(pub->fe);
	pub_font_layout(pub);

	pub->combo_count = 10;

	nk->window = SDL_CreateWindow("PMCFE", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
			pub->fe_def_size_x, pub->fe_def_size_y, SDL_WINDOW_RESIZABLE);

	if (nk->window == NULL) {

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

		SDL_Delay(10);
	}

	link_close(lp);
	config_write(pub->fe);

	free(nk);
	free(lp);
	free(pub);

	SDL_Quit();

	return 0;
}

