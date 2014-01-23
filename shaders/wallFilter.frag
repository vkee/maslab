varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	int transitions = 0;
	int blueStrips = 0;
	vec4 temp = texture(txtr,vec2(x,y),0.0);
	vec4 prev = texture(txtr,vec2(x,y-dy),0.0);
	float y_curr = y;
	if (col.x == 1){
		while (y_curr < 1){
			if (temp.y > 0.2 && prev.y < 0.1 && temp.x == 1){
				transitions++;
			}
			if (temp.x == 1 && prev.x == 0){
				blueStrips++;
			}
			prev = temp;
			y_curr += dy;
			temp = texture(txtr,vec2(x,y_curr),0.0);
		}
		if (transitions <= 1 && blueStrips == 0){
			gl_FragColor = vec4(1,0,0,1);
		}
	} else if (col.y > 0){
		gl_FragColor = vec4(0,col.y,0,1);
	}
}