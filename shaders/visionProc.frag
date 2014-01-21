varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
    final float PI = 3.1415927
    final float INCREMENT = 0.2;
    final int LOW_THRES = 0.2;
    final int HIGH_THRES = 0.8;
    final int RADIUS_THRES = 3;

	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	// BALL DETECTION RED
	int radius = 1;
	int miss = 0;
	boolean first_miss = false;
	while (radius < 50) {
		miss = 0;
		float start = sin(radius + x*y)*PI; // generate random number in [0, PI]
		for (float a = start; a < start + 2*PI; a += INCREMENT) {
			vec4 col = texture(txtr,vec2(x+float(r)*cos(a)*dx,y+float(r)*sin(a)*dy),0.0);
			if ( !(col.z==1 && col.y==0 && col.x==0) ) {
				miss++;
			}
		}
		if (miss >= (int) (2*PI/INCREMENT)*LOW_THRES) {
			if (first_miss){
				break;
			} else {
				first_miss = true;
			}
		} else {
			if (first_miss){
				first_miss = false;
				break;
			}
		}
		r++;
	}
	if (first_miss && (miss >= (int) (2*PI/INCREMENT)*HIGH_THRES) && (r >= RADIUS_THRES)) {
		gl_FragColor = vec4(0, 0, (r-1)/50.0, 1);
	}
	
	
	// BALL DETECTION GREEN
	radius = 1;
	miss = 0;
	first_miss = false;
	while (radius < 50) {
		miss = 0;
		float start = sin(radius + x*y)*PI; // generate random number in [0, PI]
		for (float a = start; a < start + 2*PI; a += INCREMENT) {
			vec4 col = texture(txtr,vec2(x+float(r)*cos(a)*dx,y+float(r)*sin(a)*dy),0.0);
			if ( !(col.z==0 && col.y==1 && col.x==0) ) {
				miss++;
			}
		}
		if (miss >= (int) (2*PI/INCREMENT)*LOW_THRES) {
			if (first_miss){
				break;
			} else {
				first_miss = true;
			}
		} else {
			if (first_miss){
				first_miss = false;
				break;
			}
		}
		r++;
	}
	if (first_miss && (miss >= (int) (2*PI/INCREMENT)*HIGH_THRES) && (r >= RADIUS_THRES)) {
		gl_FragColor = vec4(0, (r-1)/50.0, 0, 1);
	}
	
	
	// WALL DETECTION
	float yUpper = y - dy;
	float yLower = y + dy;
	vec4 col = texture(txtr,vec2(x,y),0.0);
	vec4 colUpper = texture(txtr,vec2(x,yUpper),0.0);
	vec4 colLower = texture(txtr,vec2(x,yLower),0.0);
	
	if ( col.z==0 && col.y==0 && col.x==1 ) {
		while ( colUpper.z==0 && colUpper.y==0 && colUpper.x==1 && yUpper > 0) {
			yUpper = yUpper - dy;
			colUpper = texture(txtr,vec2(x,yUpper),0.0);
		}
		
		while ( colLower.z==0 && colLower.y==0 && colLower.x==1 && yLower < 1) {
			yLower = yLower + dy;
			colLower = texture(txtr,vec2(x,yLower),0.0);
		}
		
		gl_FragColor = vec4((yLower - yUpper)*5,0,0,1);
	}
	
	/*
	float my = dy;
	vec4 col = texture(txtr,vec2(x,y),0.0);
	vec4 col2 = texture(txtr,vec2(x,y+my),0.0);
	while (my < 0.05) {
		if (col.z==0 && col.y==0 && col.x==1 && col2.z==1 && col2.y==1 && col2.x==1) {
			gl_FragColor = vec4(0,0,1,1);
			break;
		} else {
			my = my + dy;
			col2 = texture(txtr,vec2(x,y+my),0.0);
		}
	}
	*/
	
	/*
	if ( y > 0.5 ) {
		return;
	}
	
	float my = y;
	vec4 col = texture(txtr,vec2(x,my),0.0);
	vec4 col2 = texture(txtr,vec2(x,my-dy),0.0);
	if ( length(vec3(col.z-0.3,col.y-0.3,col.x-0.3))<0.02 || length(vec3(col.z-1,col.y-1,col.x-1))<0.02 ) {
		if ( col2.z==0 && col2.y==0 && col2.x==1 ) {
			gl_FragColor = vec4(0,0,1,1);
			
			while ( my > 0 ){
				vec4 col = texture(txtr,vec2(x,my),0.0);
				my -= dy;
			}
		}
	}*/
}