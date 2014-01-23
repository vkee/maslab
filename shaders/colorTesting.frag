varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	//BLUE BAND
	if ( col.x>col.y*1.2 && col.x>col.z*1.2) {
		gl_FragColor = col;//vec4(1,0,0,1);
	}
	
	//GREEN BAND
	if ( col.y>col.x*1.7 && col.y>col.z*1.7) {
		gl_FragColor = col;//vec4(0,1,0,1);
	}
	
	//RED BAND
	if ( col.z>col.x*1.7 && col.z>col.y*1.7) {
		gl_FragColor = col;//vec4(0,0,1,1);
	}
	
	//YELLOW BAND
	if ( col.y>col.x*1.2 && col.z>col.x*1.2 && col.y<col.z*1.2 && col.z<col.y*1.2) {
		gl_FragColor = vec4(0,1,1,1);
	}
}