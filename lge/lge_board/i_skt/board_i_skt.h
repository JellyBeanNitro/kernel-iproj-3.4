/* lge/lge_board/i_atnt/board_i_skt.h
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*                                                   */

#ifndef __BOARD_I_SKT_H__
#define __BOARD_I_SKT_H__
#include <mach/msm_memtypes.h>

void __init i2c_register_backlight_info(void);
void __init msm8x60_allocate_memory_regions(void);
void __init msm8x60_set_display_params(char *prim_panel, char *ext_panel);
void  msm8x60_mdp_writeback(struct memtype_reserve *reserve_table);
void __init msm_panel_init(void);
void __init msm_fb_add_devices(void);

/*                                    */
void __init i2c_register_camera_info(void);
void __init lge_camera_init(void);
void __init lge_add_btpower_devices(void);

void __init lge_add_misc_devices(void);

void __init i2c_register_input_sensor_info(void);
void __init sensor_power_init(void);

#endif
