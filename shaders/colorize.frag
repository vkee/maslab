varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	float horizon = 0.52;
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	//WALL/FLOOR
	vec4 floorSum = vec4(0,0,0,0);
	float i = 0;
	while (i <= 1) {
		floorSum = floorSum + texture(txtr,vec2(i,1),0.0);
		i = i + dx;
	}
	vec4 floorMean = normalize(floorSum);
	
	vec4 horiSum = vec4(0,0,0,0);
	float j = 0;
	while (j <= 1) {
		horiSum = horiSum + texture(txtr,vec2(i,horizon),0.0);
		j = j + dx;
	}
	vec4 horiMean = normalize(horiSum);
	if ( distance(col,horiMean) < distance(col,floorMean) ) {
		gl_FragColor = vec4(1,1,1,1);
	}
	
	
	//RED BAll
	if ( col.z>col.x*1.2 && col.z>col.y*1.2 && y > horizon)
		gl_FragColor = vec4(0,0,1,1);
	
	//GREEN BALL
	if ( col.y>col.x*1.2 && col.y>col.z*1.2 && y > horizon)
		gl_FragColor = vec4(0,1,0,1);
	
	//BLUE BAND
	if ( col.x>col.y*1.2 && col.x>col.z*1.2 && y < horizon) {
		gl_FragColor = vec4(1,0,0,1);
	}
	
	//GREEN BAND
	if ( col.y>col.x*1.2 && col.y>col.z*1.2 && y < horizon) {
		gl_FragColor = vec4(0,1,0,1);
	}
	
	//RED BAND
	if ( col.z>col.x*1.2 && col.z>col.y*1.2 && y < horizon) {
		gl_FragColor = vec4(0,0,1,1);
	}
	
	//YELLOW BAND
	if ( col.y>col.x*1.2 && col.z>col.x*1.2 && col.z<col.y*1.2 && col.y<col.z*1.2 && y < horizon) {
		gl_FragColor = vec4(0,1,1,1);
	}
	
	
	/*
	//WHITE
	if ( col.x>0.6 && col.y>0.6 && col.z>0.6 )
		gl_FragColor = vec4(256,256,256,1);
	*/	
	
	/*
	float lum = length(col)/1.73205;
	
	// WALL
	// Temporal matching
	// Uniform white expects to have low standard deviation!
	// Assume current color is the mean
	float diff = 0;
	int sz = 5;
	vec4 avg = vec4(0,0,0,0);
	for ( int j = -sz; j <= sz; j++ ) {
		avg += texture(txtr,vec2(x,y+j*dy),0.0);
	}
	avg = avg/(sz*2+1);
	for ( int j = -sz; j <= sz; j++ ) {
		vec4 other = texture(txtr,vec2(x,y+j*dy),0.0);
		diff += pow(length(other-avg),2);
	}
	float stddev = sqrt(diff/(sz*2+1));
	if ( stddev<0.03 ) {
	gl_FragColor = vec4(1,1,1,1);
	}
	
			
	// FLOOR
	// R(x) = 1.08636/(1 + pow(2,-5.43028*(x-0.583279)))
	// G(x) = 1.2038/(1 + pow(2,-4.87707*(x-0.592743)))
	// B(x) = 1.06218/(1 + pow(2,-5.70837*(x-0.497658)))
	
	float redM = 1.08636/(1 + pow(2,-5.43028*(lum-0.583279)));
	float greenM = 1.2038/(1 + pow(2,-4.87707*(lum-0.592743)));
	float blueM = 1.06218/(1 + pow(2,-5.70837*(lum-0.497658)));
	float dR = redM-col.z;
	float dG = greenM-col.y;
	float dB = blueM-col.x;
	
	if ( dR*dR+dG*dG+dB*dB<0.02 && stddev>0.02 ) {
		gl_FragColor = vec4(0.3,0.3,0.3,1);
	}
	
	// RED
	// Color Model = a/(1 + 2^(-k*(x + c)))
	// R(x) = 0.96271/(1 + pow(2,-10.5543*(x-0.343443)))
	// G(x) = 1.28698/(1 + pow(2,-4.91718*(x-0.774994)))
	// B(x) = 61.3266/(1 + pow(2,-2.64158*(x-3.34511)))
	
	redM = 0.96271/(1 + pow(2,-10.5543*(lum-0.343443)));
	greenM = 1.28698/(1 + pow(2,-4.91718*(lum-0.774994)));
	blueM = 61.3266/(1 + pow(2,-2.64158*(lum-3.34511)));
	dR = redM-col.z;
	dG = greenM-col.y;
	dB = blueM-col.x;
	
	if ( dR*dR+dG*dG+dB*dB<0.025 )
		gl_FragColor = vec4(1,0,0,1);

	// BLUE
	// Color Model = a/(1 + 2^(-k*(x + c)))
	// R(x) = 0.4/(1 + pow(2,-16.3332*(x-0.359805)))
	// G(x) = 0.543269/(1 + pow(2,-16.3332*(x-0.359805)))
	// B(x) = 6.52837/(1 + pow(2,-4.83888*(x-1.14863)))
	
	redM = 0.4/(1 + pow(2,-16.3332*(lum-0.359805)));
	greenM = 0.543269/(1 + pow(2,-16.3332*(lum-0.359805)));
	blueM = 6.52837/(1 + pow(2,-4.83888*(lum-1.14863)));
	dR = redM-col.z;
	dG = greenM-col.y;
	dB = blueM-col.x;
	
	if ( dR*dR+dG*dG+dB*dB<0.02 )
		gl_FragColor = vec4(0,0,1,1);
		
	*/
}