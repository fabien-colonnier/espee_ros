#include "espee/color_map.h"

/************************************************************************************************************
Description: create a color scale data to display
*************************************************************************************************************/
Color_maps::Color_maps(uint32_t clrmap_type, uint32_t NB_values)
{
	#ifdef VERBOSE_COLOR
 		printf("Enter color constructor\n");
	#endif
	
	NB_colors = NB_values;
	Colors_val.reserve(NB_colors);
	
	bool default_clrmap=0;
	
	if(clrmap_type ==1)    //if(!colormap_name.compare("jet"))
		default_clrmap=1;
	else if(clrmap_type ==2)  	// if(!colormap_name.compare("gray"))
	{
		uint32_t nb = 256;
		uint32_t step = round(double(nb/(NB_colors-1)));
		for(int j=0; j<(NB_colors-1); j++){
			uint32_t val = round(j*step);
			Colors_val.push_back(val*(1+256+256*256));
		}
		Colors_val.push_back(0xFFFFFF);
	}
	else
	{
		printf("colormap not supported yet, the default one (jet) will be used\n");
		default_clrmap=1;
	}
		
	if(default_clrmap)
	{
		uint32_t temp_colors[256*4+4]{};
		uint32_t i=0;
		
		//from blue to cyan
		temp_colors[i]=0x0000FF;
		while(temp_colors[i]!=0x00FFFF && i<(4*256+4-1)){
			i++;
			temp_colors[i] = temp_colors[i-1]+0x000100;
		}
		#ifdef VERBOSE_COLOR
			printf("\t blue to cyan i=%d, color = %#08x\n", i, temp_colors[i]);
		#endif
		
		//from cyan to green
		while(temp_colors[i]!=0x00FF00){
			i++;
			temp_colors[i] = temp_colors[i-1] - 0x000001;
		}
		#ifdef VERBOSE_COLOR
			printf("\t cyan to green i=%d, color = %#08x\n", i, temp_colors[i]);
		#endif
		
		//from green to yellow
		while(temp_colors[i]!=0xFFFF00){
			i++;
			temp_colors[i] = temp_colors[i-1] + 0x010000;
		}
		#ifdef VERBOSE_COLOR
			printf("\t green to yellow i=%d, color = %#08x\n", i, temp_colors[i]);
		#endif
		
		//from yellow to red
		while(temp_colors[i]!=0xFF0000){
			i++;
			temp_colors[i] = temp_colors[i-1] - 0x000100;
		}
		#ifdef verbose_Color_fcn
			printf("\t yellow to red i=%d, color = %#08x\n", i, temp_colors[i]);
		#endif
		i++;
		
		uint32_t step = round(double(i/(NB_colors-1)));
		for(int j=0; j<(NB_colors-1); j++)
			Colors_val.push_back(temp_colors[j*step]);
		
		Colors_val.push_back(temp_colors[i]);

	}
	#ifdef verbose_Color_fcn
		printf("exit color constructor\n");
	#endif
}



