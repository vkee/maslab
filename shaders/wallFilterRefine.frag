varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	vec4 temp = texture(txtr,vec2(x,y),0.0);
	
	if (col.x == 1){
		gl_FragColor = vec4(1,0,0,1);
	}
	
	temp = texture(txtr, vec2(x,y), 0.0);
	int blueNeighbor = 0;
	if (col.y > 0.2){
		for (int i = 0; i < 2; i++){
			for (int j = 0; j < 2; j++){
				temp = texture(txtr, vec2(x+(i-1)*dx,y+(j-1)*dy), 0.0);
				if (temp.x == 1){
					blueNeighbor++;
				}
			}
		}
	}
	
	if (blueNeighbor > 0){
		gl_FragColor = vec4(1,0,0,1);
	}
}