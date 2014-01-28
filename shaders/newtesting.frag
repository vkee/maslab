varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	int num_teal = 0;
	vec4 temp_col;
			
	for (int xdis = -5; xdis < 6; xdis++){
		for (int ydis = -5; ydis < 6; ydis++){
			temp_col = texture(txtr,vec2(x + xdis*dx, y + ydis*dy),0.0);
			if (temp_col.x == 1 && temp_col.y == 1 && temp_col.z == 0){
				num_teal++;
			}
		}
	}
	
	if (num_teal > 60 && y < 0.52 && y > 0.38){
		gl_FragColor = vec4(1,1,0,1);
	}
}