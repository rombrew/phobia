#ifndef _H_NK_SDL_
#define _H_NK_SDL_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define NK_ASSERT(s)			/* do nothing */

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_DEFAULT_ALLOCATOR

#include "nuklear.h"

enum {
	NK_COLOR_HIDDEN			= NK_COLOR_COUNT,
	NK_COLOR_CONFIG,
	NK_COLOR_FLICKER_LIGHT,
	NK_COLOR_FLICKER_ALERT,
	NK_COLOR_ENABLED,
	NK_COLOR_ORANGE_BUTTON,
	NK_COLOR_ORANGE_BUTTON_HOVER,
	NK_COLOR_EDIT_NUMBER,
	NK_COLOR_COMBO_HOVER,
	NK_COLOR_ACTIVE_HOVER,
	NK_COLOR_DRAWING_PEN
};

struct nk_sdl {

	struct nk_context		ctx;
	struct nk_user_font		font;
	struct nk_recti			scissor;
	struct nk_color			table[NK_COLOR_COUNT + 20];

	int				clock;
	int				updated;
	int				idled;

	SDL_Window			*window;
	SDL_Surface			*fb;
	SDL_Surface			*surface;
	TTF_Font			*ttf_font;

	int				window_ID;

	int				onquit;
	int				active;
	int				keyctrl;
};

NK_API void nk_sdl_input_event(struct nk_sdl *nk, SDL_Event *ev);
NK_API void nk_sdl_style_custom(struct nk_sdl *nk);
NK_API float nk_sdl_text_width(nk_handle font, float height, const char *text, int len);
NK_API void nk_sdl_render(struct nk_sdl *nk);

#endif /* _H_NK_SDL_ */

