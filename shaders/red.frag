varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	if ( col.z>col.y*2.3+0.1 && col.z>col.x*2.3+0.1 )
		gl_FragColor = vec4(col.z,col.y,col.x,1);
}