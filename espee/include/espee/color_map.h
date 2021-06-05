#ifndef COLOR_MAP_H
#define COLOR_MAP_H

#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <vector>

//#define VERBOSE_COLOR
#define NUM_COLORS 256  						 //number of colors used in the color gradient

class Color_maps {
	public:
		std::vector<uint32_t> Colors_val;
		uint32_t NB_colors=0;
	
		const static uint32_t ONcolor = 0x00FFFFFF;
		const static uint32_t OFFcolor = 0x00000000;
		const static uint32_t background = 0x00808080;

		Color_maps(uint32_t clrmap_type, uint32_t NB_values);		
		
	//colormap name are the same as matlab, for now 'jet', 'gray' are the only supported
	//clrmap_type 1 : jet
	//			  2 : gray
};


#endif
