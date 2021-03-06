varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/512.0;
	float dy = 1.0/512.0;
	
	
	int sz = 3;
	vec4 sum = vec4(0,0,0,0);
	float wt = 0;
	for ( int i = -sz; i <= sz; i++ ) {
		for ( int j = -sz; j <= sz; j++ ) {
			float dist = float(i*i+j*j);
			float w = pow(2,-dist/float(sz*sz));
			sum += texture(txtr,vec2(x+i*dx,y+j*dy),0.0)*w;
			wt += w;
		}
	}
	
	sum = sum*1.0/wt;
	gl_FragColor = vec4(sum.x,sum.y,sum.z,1);
}