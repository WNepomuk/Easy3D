/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include <easy3d/viewer/opengl_text.h>


namespace easy3d {

    namespace details {

        // Copyright (c) 2020 Liangliang Nan liangliang.nan@gmail.com
        // Copyright (c) 2011 Andreas Krinke andreas.krinke@gmx.de
        // Copyright (c) 2009 Mikko Mononen memon@inside.org
        //
        // This software is provided 'as-is', without any express or implied
        // warranty.  In no event will the authors be held liable for any damages
        // arising from the use of this software.
        // Permission is granted to anyone to use this software for any purpose,
        // including commercial applications, and to alter it and redistribute it
        // freely, subject to the following restrictions:
        // 1. The origin of this software must not be misrepresented; you must not
        //    claim that you wrote the original software. If you use this software
        //    in a product, an acknowledgment in the product documentation would be
        //    appreciated but is not required.
        // 2. Altered source versions must be plainly marked as such, and must not be
        //    misrepresented as being the original software.
        // 3. This notice may not be removed or altered from any source distribution.
        //
        // The significant changes are that all fixed pipeline rendering code has been
        // replaced by shader-based rendering.
        // The original code is available at https://github.com/akrinke/Font-Stash
        // The original code is available at https://github.com/armadillu/ofxFontStash


        /**
         * Example:
         *  ------------------------------------------------------------------------------------
         *  // create a font stash with a maximum texture size of 512 x 512
         *  struct sth_stash *stash = sth_create(512, 512);
         *  // load true type font
         *  const std::string font_file = resource::directory() + "/fonts/zachary.ttf";
         *  const int droid = sth_add_font(stash, font_file.c_str());
         *  ------------------------------------------------------------------------------------
         *  glEnable(GL_BLEND);
         *  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         *  // draw text during your OpenGL render loop
         *  sth_begin_draw(stash);
         *  // position: (x, y); font size: 24
         *  sth_draw_text(stash, droid, 24, x, y, "Hello world! ", &x);
         *  // now, the float x contains the x position of the next char
         *  sth_end_draw(stash);
         *  glDisable(GL_BLEND);
         *  ------------------------------------------------------------------------------------
         *  // cleaning
         *  sth_delete(stash);
         */
        struct sth_stash* sth_create(int cachew, int cacheh, int createMipmaps, int charPadding, float dpiScale);

        int sth_add_font(struct sth_stash* stash, const char* path);
        int sth_add_font_from_memory(struct sth_stash* stash, unsigned char* buffer);

        int  sth_add_bitmap_font(struct sth_stash* stash, int ascent, int descent, int line_gap);
        void sth_add_glyph(struct sth_stash* stash, int idx, unsigned int id, const char* s,  /* @rlyeh: function does not return int */
                               short size, short base, int x, int y, int w, int h,
                               float xoffset, float yoffset, float xadvance);

        void sth_begin_draw(struct sth_stash* stash);
        void sth_end_draw(struct sth_stash* stash);

        void sth_draw_text(struct sth_stash* stash,
                               int idx, float size,
                               float x, float y, const char* string, float* dx);

        void sth_dim_text(struct sth_stash* stash, int idx, float size, const char* string,
                              float* minx, float* miny, float* maxx, float* maxy);

        void sth_vmetrics(struct sth_stash* stash,
                              int idx, float size,
                              float* ascender, float* descender, float * lineh);

        void sth_delete(struct sth_stash* stash);

        void set_lod_bias(struct sth_stash* stash, float bias);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Here starts the implementation of fontstash

#include <easy3d/viewer/opengl.h>
#include <easy3d/viewer/opengl_error.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/shader_program.h>
#include <easy3d/viewer/shader_manager.h>

#define STB_TRUETYPE_IMPLEMENTATION
#include <3rd_party/stb/stb_truetype.h>


namespace easy3d {

    namespace details {

#define HASH_LUT_SIZE 256
#define MAX_ROWS 128
#define VERT_COUNT (6*128 * 100)  /* oriol! x 100 to avoid repeated flush_draw() calls while drawing */

#define TTFONT_FILE 1
#define TTFONT_MEM  2
#define BMFONT      3

        static int idx = 1;

        static unsigned int hashint(unsigned int a)
        {
            a += ~(a<<15);
            a ^=  (a>>10);
            a +=  (a<<3);
            a ^=  (a>>6);
            a += ~(a<<11);
            a ^=  (a>>16);
            return a;
        }


        struct sth_quad
        {
            float x0,y0,s0,t0;
            float x1,y1,s1,t1;
        };

        struct sth_row
        {
            short x,y,h;
        };

        struct sth_glyph
        {
            unsigned int codepoint;
            short size;
            struct sth_texture* texture;
            int x0,y0,x1,y1;
            float xadv,xoff,yoff;
            int next;
        };

        struct sth_font
        {
            int idx;
            int type;
            stbtt_fontinfo font;
            unsigned char* data;
            struct sth_glyph* glyphs;
            int lut[HASH_LUT_SIZE];
            int nglyphs;
            float ascender;
            float descender;
            float lineh;
            struct sth_font* next;
        };

        struct sth_texture
        {
            GLuint id;
            // TODO: replace rows with pointer
            struct sth_row rows[MAX_ROWS];
            int nrows;
            float verts[4*VERT_COUNT];
            int nverts;
            struct sth_texture* next;
        };

        struct sth_stash
        {
            int tw,th;
            float itw,ith;
            GLubyte *empty_data;
            struct sth_texture* tt_textures;
            struct sth_texture* bm_textures;
            struct sth_font* fonts;
            int drawing;
            int padding; //oriol adding texture padding around chars to avoid mipmap neighbor leaks
            int hasMipMap; //oriol adding optional mipmap generation to each char
            int doKerning; //calc kerning on the fly and offset letters when drawing and / calcing box sizes
            float charSpacing;
            float dpiScale;
            vec3 font_color;
        };




// Copyright (c) 2008-2009 Bjoern Hoehrmann <bjoern@hoehrmann.de>
// See http://bjoern.hoehrmann.de/utf-8/decoder/dfa/ for details.

#define UTF8_ACCEPT 0
#define UTF8_REJECT 1

        static const unsigned char utf8d[] = {
                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 00..1f
                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 20..3f
                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 40..5f
                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // 60..7f
                1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9, // 80..9f
                7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7, // a0..bf
                8,8,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2, // c0..df
                0xa,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x4,0x3,0x3, // e0..ef
                0xb,0x6,0x6,0x6,0x5,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8, // f0..ff
                0x0,0x1,0x2,0x3,0x5,0x8,0x7,0x1,0x1,0x1,0x4,0x6,0x1,0x1,0x1,0x1, // s0..s0
                1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,1, // s1..s2
                1,2,1,1,1,1,1,2,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1, // s3..s4
                1,2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1,3,1,1,1,1,1,1, // s5..s6
                1,3,1,1,1,1,1,3,1,3,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // s7..s8
        };

        static unsigned int decutf8(unsigned int* state, unsigned int* codep, unsigned int byte)
        {
            unsigned int type = utf8d[byte];
            *codep = (*state != UTF8_ACCEPT) ?
                     (byte & 0x3fu) | (*codep << 6) :
                     (0xff >> type) & (byte);
            *state = utf8d[256 + *state*16 + type];
            return *state;
        }



        struct sth_stash* sth_create(int cachew, int cacheh, int createMipmaps, int charPadding, float dpiScale){

            struct sth_stash* stash = nullptr;
            GLubyte* empty_data = nullptr;
            struct sth_texture* texture = nullptr;

            // Allocate memory for the font stash.
            stash = (struct sth_stash*)malloc(sizeof(struct sth_stash));
            if (stash == nullptr) goto error;
            memset(stash,0,sizeof(struct sth_stash));

            // Create data for clearing the textures
            empty_data = (GLubyte*)	malloc(cachew * cacheh);
            if (empty_data == nullptr) goto error;
            memset(empty_data, 0, cachew * cacheh);

            // Allocate memory for the first texture
            texture = (struct sth_texture*)malloc(sizeof(struct sth_texture));
            if (texture == nullptr) goto error;
            memset(texture,0,sizeof(struct sth_texture));

            // Create first texture for the cache.
            stash->tw = cachew;
            stash->th = cacheh;
            stash->itw = 1.0f/cachew;
            stash->ith = 1.0f/cacheh;
            stash->empty_data = empty_data;
            stash->tt_textures = texture;
            stash->dpiScale = dpiScale;
            glGenTextures(1, &texture->id); easy3d_debug_log_gl_error;
            if (!texture->id) goto error;
            glBindTexture(GL_TEXTURE_2D, texture->id);    easy3d_debug_log_gl_error;
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);    easy3d_debug_log_gl_error;
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);    easy3d_debug_log_gl_error;
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, cachew, cacheh, 0, GL_RED, GL_UNSIGNED_BYTE, empty_data);    easy3d_debug_log_gl_error;
            glBindTexture(GL_TEXTURE_2D, 0);    easy3d_debug_log_gl_error;

            stash->hasMipMap = createMipmaps;
            stash->padding = charPadding;
            stash->charSpacing = 0.0f;
            stash->doKerning = 0;
            return stash;

            error:
            if (stash != nullptr)
                free(stash);
            if (texture != nullptr)
                free(texture);
            return nullptr;
        }

        int sth_add_font_from_memory(struct sth_stash* stash, unsigned char* buffer)
        {
            int i, ascent, descent, fh, lineGap;
            struct sth_font* fnt = nullptr;

            fnt = (struct sth_font*)malloc(sizeof(struct sth_font));
            if (fnt == nullptr) goto error;
            memset(fnt,0,sizeof(struct sth_font));

            // Init hash lookup.
            for (i = 0; i < HASH_LUT_SIZE; ++i) fnt->lut[i] = -1;

            fnt->data = buffer;


            // Init stb_truetype
            if (!stbtt_InitFont(&fnt->font, fnt->data, 0)) goto error;

            // Store normalized line height. The real line height is got
            // by multiplying the lineh by font size.
            stbtt_GetFontVMetrics(&fnt->font, &ascent, &descent, &lineGap);
            fh = ascent - descent;
            fnt->ascender = (float)ascent / (float)fh;
            fnt->descender = (float)descent / (float)fh;
            fnt->lineh = (float)(fh + lineGap) / (float)fh;

            fnt->idx = idx;
            fnt->type = TTFONT_MEM;
            fnt->next = stash->fonts;
            stash->fonts = fnt;

            return idx++;

            error:
            if (fnt) {
                if (fnt->glyphs) free(fnt->glyphs);
                free(fnt);
            }
            return 0;
        }

        int sth_add_font(struct sth_stash* stash, const char* path)
        {
            FILE* fp = 0;
            int datasize;
            unsigned char* data = nullptr;
            int idx;

            // Read in the font data.
            fp = fopen(path, "rb");
            if (!fp) goto error;
            fseek(fp,0,SEEK_END);
            datasize = (int)ftell(fp);
            fseek(fp,0,SEEK_SET);
            data = (unsigned char*)malloc(datasize);
            if (data == nullptr) goto error;
            fread(data, 1, datasize, fp);
            fclose(fp);
            fp = 0;

            idx = sth_add_font_from_memory(stash, data);
            // Modify type of the loaded font.
            if (idx)
                stash->fonts->type = TTFONT_FILE;
            else
                free(data);

            return idx;

            error:
            if (data) free(data);
            if (fp) fclose(fp);
            return 0;
        }

        int sth_add_bitmap_font(struct sth_stash* stash, int ascent, int descent, int line_gap)
        {
            int i, fh;
            struct sth_font* fnt = nullptr;

            fnt = (struct sth_font*)malloc(sizeof(struct sth_font));
            if (fnt == nullptr) goto error;
            memset(fnt,0,sizeof(struct sth_font));

            // Init hash lookup.
            for (i = 0; i < HASH_LUT_SIZE; ++i) fnt->lut[i] = -1;

            // Store normalized line height. The real line height is got
            // by multiplying the lineh by font size.
            fh = ascent - descent;
            fnt->ascender = (float)ascent / (float)fh;
            fnt->descender = (float)descent / (float)fh;
            fnt->lineh = (float)(fh + line_gap) / (float)fh;

            fnt->idx = idx;
            fnt->type = BMFONT;
            fnt->next = stash->fonts;
            stash->fonts = fnt;

            return idx++;

            error:
            if (fnt) free(fnt);
            return 0;
        }

        void sth_add_glyph(struct sth_stash* stash,
                               int idx,
                               GLuint id,
                               const char* s,
                               short size, short base,
                               int x, int y, int w, int h,
                               float xoffset, float yoffset, float xadvance)
        {
            struct sth_texture* texture = nullptr;
            struct sth_font* fnt = nullptr;
            struct sth_glyph* glyph = nullptr;
            unsigned int codepoint;
            unsigned int state = 0;

            if (stash == nullptr) return;
            texture = stash->bm_textures;
            while (texture != nullptr && texture->id != id) texture = texture->next;
            if (texture == nullptr)
            {
                // Create new texture
                texture = (struct sth_texture*)malloc(sizeof(struct sth_texture));
                if (texture == nullptr) return;
                memset(texture, 0, sizeof(struct sth_texture));
                texture->id = id;
                texture->next = stash->bm_textures;
                stash->bm_textures = texture;
            }

            fnt = stash->fonts;
            while (fnt != nullptr && fnt->idx != idx) fnt = fnt->next;
            if (fnt == nullptr) return;
            if (fnt->type != BMFONT) return;

            for (; *s; ++s)
            {
                if (!decutf8(&state, &codepoint, *(unsigned char*)s)) break;
            }
            if (state != UTF8_ACCEPT) return;

            // Alloc space for new glyph.
            fnt->nglyphs++;
            fnt->glyphs = (struct sth_glyph *)realloc(fnt->glyphs, fnt->nglyphs*sizeof(struct sth_glyph)); /* @rlyeh: explicit cast needed in C++ */
            if (!fnt->glyphs) return;

            // Init glyph.
            glyph = &fnt->glyphs[fnt->nglyphs-1];
            memset(glyph, 0, sizeof(struct sth_glyph));
            glyph->codepoint = codepoint;
            glyph->size = size;
            glyph->texture = texture;
            glyph->x0 = x;
            glyph->y0 = y;
            glyph->x1 = glyph->x0+w;
            glyph->y1 = glyph->y0+h;
            glyph->xoff = xoffset;
            glyph->yoff = yoffset - base;
            glyph->xadv = xadvance;

            // Find code point and size.
            h = hashint(codepoint) & (HASH_LUT_SIZE-1);
            // Insert char to hash lookup.
            glyph->next = fnt->lut[h];
            fnt->lut[h] = fnt->nglyphs-1;
        }

        static struct sth_glyph* get_glyph(struct sth_stash* stash, struct sth_font* fnt, unsigned int codepoint, short isize)
        {
            int i,g,advance,lsb,x0,y0,x1,y1,gw,gh;
            float scale;
            struct sth_texture* texture = nullptr;
            struct sth_glyph* glyph = nullptr;
            unsigned char* bmp = nullptr;
            unsigned int h;
            float size = isize/10.0f;
            int rh;
            struct sth_row* br = nullptr;

            // Find code point and size.
            h = hashint(codepoint) & (HASH_LUT_SIZE-1);
            i = fnt->lut[h];
            while (i != -1)
            {
                if (fnt->glyphs[i].codepoint == codepoint && (fnt->type == BMFONT || fnt->glyphs[i].size == isize))
                    return &fnt->glyphs[i];
                i = fnt->glyphs[i].next;
            }
            // Could not find glyph.

            // For bitmap fonts: ignore this glyph.
            if (fnt->type == BMFONT) return 0;

            // For truetype fonts: create this glyph.
            scale = stash->dpiScale * stbtt_ScaleForPixelHeight(&fnt->font, size);
            g = stbtt_FindGlyphIndex(&fnt->font, codepoint);
            if(!g) return 0; /* @rlyeh: glyph not found, ie, arab chars */
            stbtt_GetGlyphHMetrics(&fnt->font, g, &advance, &lsb);
            stbtt_GetGlyphBitmapBox(&fnt->font, g, scale,scale, &x0,&y0,&x1,&y1);

            gw = x1-x0 + stash->padding;
            gh = y1-y0 + stash->padding;

            // Check if glyph is larger than maximum texture size
            if (gw >= stash->tw || gh >= stash->th)
                return 0;

            // Find texture and row where the glyph can be fit.
            br = nullptr;
            rh = (gh+7) & ~7;
            texture = stash->tt_textures;
            while(br == nullptr)
            {
                for (i = 0; i < texture->nrows; ++i)
                {
                    if (texture->rows[i].h == rh && texture->rows[i].x+gw+1 <= stash->tw)
                        br = &texture->rows[i];
                }

                // If no row is found, there are 3 possibilities:
                //   - add new row
                //   - try next texture
                //   - create new texture
                if (br == nullptr)
                {
                    short py = 0;
                    // Check that there is enough space.
                    if (texture->nrows)
                    {
                        py = texture->rows[texture->nrows-1].y + texture->rows[texture->nrows-1].h+1;
                        if (py+rh > stash->th)
                        {
                            if (texture->next != nullptr)
                            {
                                texture = texture->next;
                            }
                            else
                            {
                                // Create new texture
                                texture->next = (struct sth_texture*)malloc(sizeof(struct sth_texture));
                                texture = texture->next;
                                if (texture == nullptr) goto error;
                                memset(texture,0,sizeof(struct sth_texture));
                                //oriol counting how many we have created so far!
                                int numTex = 1;
                                struct sth_texture* tex = stash->tt_textures;
                                while (tex->next != nullptr) {
                                    numTex++;
                                    tex = tex->next;
                                }

                                LOG(INFO) << "allocating a new texture of " << stash->tw << " x " << stash->th << " (" << numTex << " used so far)";
                                glGenTextures(1, &texture->id);
                                if (!texture->id) goto error;
                                glBindTexture(GL_TEXTURE_2D, texture->id);
                                glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, stash->tw,stash->th, 0, GL_RED, GL_UNSIGNED_BYTE, stash->empty_data); easy3d_debug_log_gl_error;
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); easy3d_debug_log_gl_error;
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); easy3d_debug_log_gl_error;
                                glBindTexture(GL_TEXTURE_2D, 0);
                            }
                            continue;
                        }
                    }
                    // Init and add row
                    br = &texture->rows[texture->nrows];
                    br->x = 0;
                    br->y = py;
                    br->h = rh;
                    texture->nrows++;
                }
            }

            // Alloc space for new glyph.
            fnt->nglyphs++;
            fnt->glyphs = (struct sth_glyph *)realloc(fnt->glyphs, fnt->nglyphs*sizeof(struct sth_glyph)); /* @rlyeh: explicit cast needed in C++ */
            if (!fnt->glyphs) return 0;

            // Init glyph.
            glyph = &fnt->glyphs[fnt->nglyphs-1];
            memset(glyph, 0, sizeof(struct sth_glyph));
            glyph->codepoint = codepoint;
            glyph->size = isize;
            glyph->texture = texture;
            glyph->x0 = br->x;
            glyph->y0 = br->y;
            glyph->x1 = glyph->x0+gw;
            glyph->y1 = glyph->y0+gh;
            glyph->xadv = scale * advance;
            glyph->xoff = (float)x0;
            glyph->yoff = (float)y0;
            glyph->next = 0;

            // Advance row location.
            br->x += gw+1;

            // Insert char to hash lookup.
            glyph->next = fnt->lut[h];
            fnt->lut[h] = fnt->nglyphs-1;

            // Rasterize
            bmp = (unsigned char*)malloc(gw*gh);
            if (bmp)
            {
                stbtt_MakeGlyphBitmap(&fnt->font, bmp, gw,gh,gw, scale,scale, g);
                // Update texture
                glBindTexture(GL_TEXTURE_2D, texture->id);   easy3d_debug_log_gl_error;
                glPixelStorei(GL_UNPACK_ALIGNMENT,1);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);	//oriol trying to get rid of halos when rotating font
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);	//
                glTexSubImage2D(GL_TEXTURE_2D, 0, glyph->x0,glyph->y0, gw,gh, GL_RED,GL_UNSIGNED_BYTE,bmp);
                if(stash->hasMipMap > 0){
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, 8); //TODO check for hw support!
#if defined(__ANDROID__) || defined(TARGET_OPENGLES) || defined(TARGET_RASPBERRY_PI)
                    // OpenGLES 1.0 does not support the following.
#else
//			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, -0.0); //shoot for sharper test
//			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 3); // pick mipmap level 7 or lower
//			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, -0.0);
                    glGenerateMipmap(GL_TEXTURE_2D);
#endif
                }
                glBindTexture(GL_TEXTURE_2D, 0);   easy3d_debug_log_gl_error;
                free(bmp);
            }

            return glyph;

            error:
            if (texture)
                free(texture);
            return 0;
        }

        static int get_quad(struct sth_stash* stash, struct sth_font* fnt, struct sth_glyph* glyph, short isize, float* x, float* y, struct sth_quad* q)
        {
            int rx,ry;
            float scale = 1.0f ;

            if (fnt->type == BMFONT) scale = isize/(glyph->size*10.0f);

            rx = floorf(*x + scale * glyph->xoff);
            ry = floorf(*y - scale * glyph->yoff);

            q->x0 = rx;
            q->y0 = ry;
            q->x1 = rx + scale * (glyph->x1 - glyph->x0);
            q->y1 = ry - scale * (glyph->y1 - glyph->y0);

            q->s0 = (glyph->x0) * stash->itw;
            q->t0 = (glyph->y0) * stash->ith;
            q->s1 = (glyph->x1) * stash->itw;
            q->t1 = (glyph->y1) * stash->ith;

            *x += scale * glyph->xadv;

            return 1;
        }

        static float* setv(float* v, float x, float y, float s, float t)
        {
            v[0] = x;
            v[1] = y;
            v[2] = s;
            v[3] = t;
            return v+4;
        }


        void set_lod_bias(struct sth_stash* stash, float bias){

            struct sth_texture* texture = stash->tt_textures;
            if(stash->hasMipMap > 0){
                while (texture){
                    glBindTexture(GL_TEXTURE_2D, texture->id);
#ifndef TARGET_OPENGLES
                    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, bias);
#endif
                    glBindTexture(GL_TEXTURE_2D, 0);
                    texture = texture->next;
                }
            }
        }

        static void flush_draw(struct sth_stash* stash)
        {
            struct sth_texture* texture = stash->tt_textures;
            short tt = 1;
            while (texture)
            {
                if (texture->nverts > 0)
                {
                    int viewport[4];
                    glGetIntegerv(GL_VIEWPORT, viewport);
                    const int w = viewport[2];
                    const int h = viewport[3];

                    std::vector<vec3> verts(texture->nverts);
                    std::vector<vec2> texcoords(texture->nverts);
                    for (int i=0;i<texture->nverts; ++i) {
                        float x = texture->verts[i * 4];
                        float y = texture->verts[i * 4 + 1];
                        verts[i] = vec3(2.0f * x / w - 1.0f, 2.0f * y / h - 1.0f, -0.9f);
                        texcoords[i] = vec2(texture->verts + i * 4 + 2);
                    }

                    std::vector<unsigned int> indices; // = { 0, 1, 2, 0, 2, 3 };
                    for (int j = 0; j < texture->nverts/4; ++j) {
                        indices.push_back(j * 4);
                        indices.push_back(j * 4 + 1);
                        indices.push_back(j * 4 + 2);
                        indices.push_back(j * 4);
                        indices.push_back(j * 4 + 2);
                        indices.push_back(j * 4 + 3);
                    }

                    TrianglesDrawable drawable;
                    drawable.update_vertex_buffer(verts);
                    drawable.update_texcoord_buffer(texcoords);
                    drawable.update_index_buffer(indices);

                    const std::string name = "text/text";
                    auto program = ShaderManager::get_program(name);
                    if (!program) {
                        std::vector<ShaderProgram::Attribute> attributes = {
                                ShaderProgram::Attribute(ShaderProgram::POSITION, "vtx_position"),
                                ShaderProgram::Attribute(ShaderProgram::TEXCOORD, "tex_coord")
                        };
                        program = ShaderManager::create_program_from_files(name, attributes);
                    }
                    if (!program) {
                        LOG_FIRST_N(ERROR, 1) << "shader doesn't exist: " << name << " (this is the first record)";
                        return;
                    }

                    program->bind();
                    program->bind_texture("textureID", texture->id, 0)->set_uniform("font_color", stash->font_color);
                    drawable.gl_draw(false);
                    program->release_texture();
                    program->release();

                    texture->nverts = 0;
                }
                texture = texture->next;
                if (!texture && tt)
                {
                    texture = stash->bm_textures;
                    tt = 0;
                }
            }
        }

        void sth_begin_draw(struct sth_stash* stash)
        {
            if (stash == nullptr) return;
            if (stash->drawing)
                flush_draw(stash);
            stash->drawing = 1;
        }

        void sth_end_draw(struct sth_stash* stash)
        {
            if (stash == nullptr) return;
            if (!stash->drawing) return;

            flush_draw(stash);
            stash->drawing = 0;
        }

        void sth_draw_text(struct sth_stash* stash,
                               int idx, float size,
                               float x, float y,
                               const char* s, float* dx)
        {
            unsigned int codepoint;
            struct sth_glyph* glyph = nullptr;
            struct sth_texture* texture = nullptr;
            unsigned int state = 0;
            struct sth_quad q;
            short isize = (short)(size*10.0f);
            float* v;
            struct sth_font* fnt = nullptr;

            if (stash == nullptr) return;

            fnt = stash->fonts;
            while(fnt != nullptr && fnt->idx != idx) fnt = fnt->next;
            if (fnt == nullptr) return;
            if (fnt->type != BMFONT && !fnt->data) return;

            int len = strlen(s);
            float scale = stbtt_ScaleForPixelHeight(&fnt->font, size);
            int c = 0;
            float spacing = stash->charSpacing;
            int doKerning = stash->doKerning;
            int p = stash->padding;
            float dpiScale = stash->dpiScale;
            float tw = stash->padding / (float)stash->tw;

            for (; *s; ++s)
            {
                if (decutf8(&state, &codepoint, *(unsigned char*)s)) continue;
                glyph = get_glyph(stash, fnt, codepoint, isize);
                if (!glyph) continue;
                texture = glyph->texture;
                if (texture->nverts+4 >= VERT_COUNT)
                    flush_draw(stash);

                if (!get_quad(stash, fnt, glyph, isize, &x, &y, &q)) continue;

                int diff = 0;
                if (c < len && doKerning > 0){
                    diff = stbtt_GetCodepointKernAdvance(&fnt->font, *(s), *(s+1));
                    //printf("diff '%c' '%c' = %d\n", *(s-1), *s, diff);
                    x += diff * scale;
                }
                x += dpiScale * spacing;

                v = &texture->verts[texture->nverts*4];

                v = setv(v, q.x0, q.y0, q.s0, q.t0);
                v = setv(v, q.x1, q.y0, q.s1, q.t0);
                v = setv(v, q.x1, q.y1, q.s1, q.t1);
                v = setv(v, q.x0, q.y1, q.s0, q.t1);

                texture->nverts += 4;
                c++;
            }

            if (dx) *dx = x / dpiScale;
        }

        void sth_dim_text(struct sth_stash* stash,
                              int idx, float size,
                              const char* s,
                              float* minx, float* miny, float* maxx, float* maxy)
        {
            unsigned int codepoint;
            struct sth_glyph* glyph = nullptr;
            unsigned int state = 0;
            struct sth_quad q;
            short isize = (short)(size*10.0f);
            struct sth_font* fnt = nullptr;
            float x = 0, y = 0;

            *minx = *maxx = *miny = *maxy = 0;	/* @rlyeh: reset vars before failing */

            if (stash == nullptr) return;
            fnt = stash->fonts;
            while(fnt != nullptr && fnt->idx != idx) fnt = fnt->next;
            if (fnt == nullptr) return;
            if (fnt->type != BMFONT && !fnt->data) return;

            int len = strlen(s);
            float scale = stbtt_ScaleForPixelHeight(&fnt->font, size);
            int c = 0;
            float spacing = stash->charSpacing;
            int doKerning = stash->doKerning;
            float dpiScale = stash->dpiScale;

            for (; *s; ++s){
                if (decutf8(&state, &codepoint, *(unsigned char*)s)) continue;
                glyph = get_glyph(stash, fnt, codepoint, isize);
                if (!glyph) continue;
                if (!get_quad(stash, fnt, glyph, isize, &x, &y, &q)) continue;

                int diff = 0;
                if (c < len && doKerning > 0){
                    diff = stbtt_GetCodepointKernAdvance(&fnt->font, *(s), *(s+1));
                    //printf("diff '%c' '%c' = %d\n", *(s-1), *s, diff);
                    x += diff * scale;
                }
                x += spacing;

                if (q.x0 < *minx) *minx = q.x0;
                if (q.x1 > *maxx) *maxx = q.x1;
                if (q.y1 < *miny) *miny = q.y1;
                if (q.y0 > *maxy) *maxy = q.y0;
                c++;
            }
            if (x > *maxx) *maxx = x;
        }

        void sth_vmetrics(struct sth_stash* stash,
                              int idx, float size,
                              float* ascender, float* descender, float* lineh)
        {
            struct sth_font* fnt = nullptr;

            if (stash == nullptr) return;
            fnt = stash->fonts;
            while(fnt != nullptr && fnt->idx != idx) fnt = fnt->next;
            if (fnt == nullptr) return;
            if (fnt->type != BMFONT && !fnt->data) return;
            if (ascender)
                *ascender = fnt->ascender*size;
            if (descender)
                *descender = fnt->descender*size;
            if (lineh)
                *lineh = fnt->lineh*size;
        }

        void sth_delete(struct sth_stash* stash)
        {
            struct sth_texture* tex = nullptr;
            struct sth_texture* curtex = nullptr;
            struct sth_font* fnt = nullptr;
            struct sth_font* curfnt = nullptr;

            if (!stash) return;

            tex = stash->tt_textures;
            while(tex != nullptr) {
                curtex = tex;
                tex = tex->next;
                if (curtex->id)
                    glDeleteTextures(1, &curtex->id);
                free(curtex);
            }

            tex = stash->bm_textures;
            while(tex != nullptr) {
                curtex = tex;
                tex = tex->next;
                if (curtex->id)
                    glDeleteTextures(1, &curtex->id);
                free(curtex);
            }

            fnt = stash->fonts;
            while(fnt != nullptr) {
                curfnt = fnt;
                fnt = fnt->next;
                if (curfnt->glyphs)
                    free(curfnt->glyphs);
                if (curfnt->type == TTFONT_FILE && curfnt->data)
                    free(curfnt->data);
                free(curfnt);
            }
            free(stash->empty_data);
            free(stash);
        }
    }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Here starts the implementation of OpenGLText

#include <easy3d/util/logging.h>


namespace easy3d {


    OpenGLText::OpenGLText(float dpi_scale, int texture_size, bool mipmaps)
        : dpi_scale_(dpi_scale)
    {
        texture_size_ = geom::next_pow2(texture_size);
        stash_ = details::sth_create(texture_size_, texture_size_, mipmaps, 0, dpi_scale); easy3d_log_gl_error;
        if (stash_ == nullptr) {
            LOG(ERROR) << "construction of OpenGLText failed";
        }
        else {
            stash_->doKerning = 0; //kerning disabled by default
            stash_->charSpacing = 0.0; //spacing neutral by default
        }
    }

    OpenGLText::~OpenGLText() {
        if (stash_ != nullptr)
            sth_delete(stash_);
    }

    bool OpenGLText::add_font(const std::string &font_file) {
        if (stash_ == nullptr) {
            LOG(ERROR) << "construction of OpenGLText failed";
            return false;
        }

        LOG(INFO) << "loading font '" << font_file << "' in texture (" << texture_size_ << " x " << texture_size_ << ")";
        const int id = sth_add_font(stash_, font_file.c_str());
        if (id <= 0) {
            LOG(ERROR) << "could not load font: " << font_file;
            return false;
        }

        font_ids_.push_back(id);
        return true;
    }


    float OpenGLText::draw(const std::string &text, float x, float y, float font_size, int fontID,
                           const vec3 &font_color) const {

        if (true) { // upper_left corner is the origin
            int viewport[4];
            glGetIntegerv(GL_VIEWPORT, viewport);
            const int h = viewport[3];
            y = h - y - 1 - font_height(font_size);
        }

        float endx = 0.0f;
        if (stash_ != nullptr) {
            stash_->font_color = font_color;
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            sth_begin_draw(stash_); easy3d_debug_log_gl_error;
            sth_draw_text(stash_, font_ids_[fontID], font_size, x, y, text.c_str(), &endx); //this might draw
            easy3d_debug_log_gl_error;
            sth_end_draw(stash_); // this actually draws
            easy3d_debug_log_gl_error;
            glDisable(GL_BLEND);
        } else {
            LOG(ERROR) << "couldn't draw() due to the failure in initialization";
        }
        return endx;
    }


#ifdef ENABLE_MULTILINE_TEXT_RENDERING

#define OFX_FONT_STASH_LINE_HEIGHT_MULT	0.9

    Rect OpenGLText::draw_multi_line(const std::string &text, float x0, float y0, float font_size, Align align, float width, int fontID, const vec3 &font_color) const {
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        const int h = viewport[3];

        Rect area(0, 0, 0, 0);
        if (stash_ != NULL){
            std::stringstream ss(text);
            std::string s;
            int line = 0;
            std::vector<std::string> lines;
            std::vector<float> widths;
            std::vector<float> ys;
            float maxW = width;

            const float lineHeight = 1.0f; // as percent, 1.0 would be normal

            while ( getline(ss, s, '\n') ) {
                lines.push_back(s);
                float yy = font_size * lineHeight * OFX_FONT_STASH_LINE_HEIGHT_MULT * line * stash_->dpiScale;
                ys.push_back(yy);
                Rect dim = get_bbox(s, font_size, x0, y0 + yy / stash_->dpiScale, ALIGN_LEFT, 0.0f);

                if(line == 0){
                    area = dim;
                } else{
                    area = Rect(std::min(area.x_min(), dim.x_min()),
                                std::max(area.x_max(), dim.x_max()),
                                std::min(area.y_min(), dim.y_min()),
                                std::max(area.y_max(), dim.y_max()));
                }
                widths.push_back(dim.width());
                if(width == 0){
                    if(maxW < dim.width()) maxW = dim.width();
                }
                line++;
            }

            stash_->font_color = font_color;
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            sth_begin_draw(stash_);
            {
                float yy = font_size * lineHeight * OFX_FONT_STASH_LINE_HEIGHT_MULT * line * stash_->dpiScale;
                float minDiffX = FLT_MAX;
                for(int i = 0; i < lines.size(); i++){
                    float dx = 0;
                    float x = 0;
                    switch (align) {
                        case ALIGN_LEFT: break;
                        case ALIGN_RIGHT: x = maxW - widths[i]; break;
                        case ALIGN_CENTER: x = (maxW - widths[i]) * 0.5; break;
                        default: break;
                    }
                    if(minDiffX > x) minDiffX = x;
                    sth_draw_text(stash_,
                                  font_ids_[fontID],
                                  font_size,
                                  x0 + x * stash_->dpiScale,
                                  h - ys[i] - 1 - font_height(font_size) - y0,
                                  lines[i].c_str(),
                                  &dx
                    );
                }
                area.x() += minDiffX;
            }
            sth_end_draw(stash_);
            easy3d_debug_log_gl_error;
            glDisable(GL_BLEND);
        } else {
            LOG(ERROR) << "couldn't draw() due to the failure in initialization";
        }

        return area;
    }


    Rect OpenGLText::get_bbox(const std::string& text, float font_size, float xx, float yy, Align align, float width) const {

        Rect totalArea(0, 0, 0, 0);
        const float lineHeight = 1.0f; // as percent, 1.0 would be normal

        if (stash_ != NULL){
            std::stringstream ss(text);
            std::string s;
            int line = 0;
            float totalH = 0;
            std::vector<Rect> rects;
            while ( getline(ss, s, '\n') ) {

                float dx = 0;
                float w, h, x, y;
                sth_dim_text( stash_, font_ids_[0], font_size / stash_->dpiScale, s.c_str(), &x, &y, &w, &h);

                totalArea.x() = x + xx;
                totalArea.y() = yy + y ;
                w = fabs (w - x);
                h = fabs (y - h);
                if(w > totalArea.width()) totalArea.x_max() = totalArea.x() + w;
                if(h > totalArea.height()) totalArea.y_max() = totalArea.y() + h;
                Rect r2 = totalArea;
                r2.y() -= r2.height();
                r2.y() += ((font_size * lineHeight)) * OFX_FONT_STASH_LINE_HEIGHT_MULT * line;
                rects.push_back(r2);

                line ++;
            }

            if(line > 1){ //if multiline
                totalArea.y() -= rects[0].height();
            }else{
                totalArea.y() -= totalArea.height();
            }

        } else {
            LOG(ERROR) << "couldn't draw() due to the failure in initialization";
        }

//        if(extraPadding > 0){
//            totalArea.width -= extraPadding;
//            totalArea.height -= extraPadding;
//        }

        if(align != ALIGN_LEFT){
            if(align == ALIGN_RIGHT){
                totalArea.x() += width - totalArea.width();
            }else{
                totalArea.x() += (width - totalArea.width()) * 0.5;
            }
        }

        return totalArea;
    }
#endif

    void OpenGLText::set_character_spacing(float spacing) {
        if (stash_)
            stash_->charSpacing = spacing;
    }


    float OpenGLText::character_spacing() const {
        if (stash_)
            return stash_->charSpacing;
        else
            return 0.0f;
    }


    void OpenGLText::set_kerning(bool kerning) {
        if (stash_)
            stash_->doKerning = kerning;
    }


    bool OpenGLText::kerning() const {
        if (stash_)
            return stash_->doKerning;
        else
            return false;
    }


    float OpenGLText::font_height(float font_size) const {
        float asc, desc, lineh;
        sth_vmetrics(stash_, font_ids_[0], font_size, &asc, &desc, &lineh);
        return asc - desc;
    }

}