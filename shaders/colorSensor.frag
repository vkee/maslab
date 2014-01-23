varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/32.0;
	float dy = 1.0/24.0;
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	//RED BAll
	if ( col.z>col.x*1.7 && col.z>col.y*1.7)
		gl_FragColor = vec4(0,0,1,1);
	
	//GREEN BALL
	if ( col.y>col.x*1.7 && col.y>col.z*1.7)
		gl_FragColor = vec4(0,1,0,1);
}