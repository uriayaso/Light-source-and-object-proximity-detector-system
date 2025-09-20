#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;

void main(void){
	
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();
  lcd_init();
  lcd_clear();
  
  while(1){
	switch(state){
		
	  case state0:
        enterLPM(lpm_mode);
		break;
		 
	  case state1:
	  	lcd_clear();
		state1Func();
		state = state0;
		break;
		 
	  case state2:
	  	lcd_clear();
		state2Func();
		state = state0;
		break;
		
		
	  case state3:
	  	lcd_clear();
		state3Func();
		state = state0;
		break;

	  case state4:
	  	lcd_clear();
		state4Func();
		state = state0;		
		break;

	  case state5:
	  	lcd_clear();
		state5Func();
		break;

	  case state6:
		lcd_clear();
		calibration();
		state = state0;
		break;		
	}
  }
}
  
  
  

  
  