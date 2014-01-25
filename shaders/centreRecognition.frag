varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	float horizon = 0.52;
	float maxRadii = 0.0;
	float maxX = 0.0;
	float maxY = 0.0;
	float tempX = 0.0;
	float tempY = 0.0;
	
	if (x < dx && y < dy) {
		while (tempX < 1.0) {
			while (tempY < 1.0) {
				vec4 colTemp = texture(txtr,vec2(tempX,tempY),0.0);
				if (tempY>horizon && colTemp.y>maxRadii) {
					maxRadii = colTemp.y;
					maxX = tempX;
					maxY = tempY;
				}
				
				if (tempY>horizon && colTemp.z>maxRadii) {
					maxRadii = colTemp.z;
					maxX = tempX;
					maxY = tempY;
				}
				
				tempY = tempY + dy;
			}
			tempY = 0.5;
			tempX = tempX + dx;
		}
		gl_FragColor = vec4(maxX,maxY,maxRadii,1);
		//gl_FragColor = vec4(1,1,1,1);
	} else {
		gl_FragColor = texture(txtr,vec2(x,y),0.0);
	}
}