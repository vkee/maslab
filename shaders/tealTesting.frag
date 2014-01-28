varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	//TEAL BAND
	if ( col.y>col.z*1.5 && col.x>col.z*1.5 && col.y<col.x*1.3 && col.x<col.y*1.3) {
		gl_FragColor = vec4(1,1,0,1);
	}
}