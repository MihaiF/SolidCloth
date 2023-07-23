#version 400

layout(location = 0) out vec4 out_color;

uniform vec3 light_position;
uniform vec3 light_dir;
uniform vec3 eye_position;
uniform int material_shininess;
uniform vec3 material_kd;
uniform vec3 material_ks;
uniform int flip;
uniform int flags;

uniform sampler2D map_color;

#define PCF
#ifdef PCF
uniform sampler2DShadow shadowMap;
#else
uniform sampler2D shadowMap; // normal texture for shadow map
#endif

#define VERTEX_COLORS 1
#define OREN_NAYAR 2
#define TEXTURED 4
#define MONOCHROME 8
#define WIREFRAME 16
#define SHADOWS 32

in vec3 color;
in vec4 ShadowCoord;
in vec3 dist;

in VertexData
{
	vec3 p;
	vec3 n;
	vec2 uv;
} fs_in;

vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

// Returns a random number based on a vec3 and an int.
float random(vec3 seed, int i){
	vec4 seed4 = vec4(seed,i);
	float dot_product = dot(seed4, vec4(12.9898,78.233,45.164,94.673));
	return fract(sin(dot_product) * 43758.5453);
}

float orennayar(float roughness){
	vec3 position = fs_in.p;
	vec3 normal = fs_in.n;
	
    vec3 L = normalize(light_position - position);
    vec3 N = normalize(normal);
    vec3 V = normalize(eye_position - position);

    float acosVN = acos(dot(V, N));
    float acosLN = acos(dot(L, N));
    float alpha = max(acosVN, acosLN);
    float beta = min(acosVN, acosLN);
    float gamma = dot(V - N * dot(V, N), L - N * dot(L, N));
    float rough_sq = roughness * roughness;

    float C1 = 1.0f - 0.5f * (rough_sq / (rough_sq + 0.33f));
    float C2 = 0.45f * (rough_sq / (rough_sq + 0.09));
    if (gamma >= 0)
    {
        C2 *= sin(alpha);
    }
    else
    {
        C2 *= (sin(alpha) - pow((2 * beta) / 3.14167, 3));
    }

    float C3 = (1.0f / 8.0f);
    C3 *= (rough_sq / (rough_sq + 0.09f));
    C3 *= pow((4.0f * alpha * beta) / (3.14167 * 3.14167), 2);

    float A = gamma * C2 * tan(beta);
    float B = (1 - abs(gamma)) * C3 * tan((alpha + beta) / 2.0f);

    return min(max(0.0f, dot(N, L)) * (C1 + A + B),0.99f);
}

float CalculateShadow(float cosTheta)
{
	vec3 position = fs_in.p;
	// Biased shadow coordinates; TODO: what about division by w?
	vec4 shadowCoords = vec4(ShadowCoord.xyz * 0.5 + vec3(0.5), 1);
	
	// cosTheta is dot( n,l ), clamped between 0 and 1
	//float tanTheta = tan(acos(cosTheta));
	float tanTheta = sqrt(1.0f / (cosTheta * cosTheta) - 1.0f);
	float bias = 0.005f;
	bias = clamp(bias * tanTheta, 0.0f, 0.008f);
	float visibility = 1.0f;
	
#ifndef PCF
	// Shadows using a traditional texture
	float depth = texture(shadowMap, shadowCoords.xy).r;
	if (depth  <  (shadowCoords.z - bias) / shadowCoords.w) {
		visibility = 0.5;
	}
#else
	// Compute the shadow visibility factor
	//visibility = max(texture(shadowMap, vec3(shadowCoords.xy, (shadowCoords.z - bias) / shadowCoords.w)), 0.5f);
	
	// Sample the shadow map 4 times
	for (int i=0;i<4;i++){
		// use either :
		//  - Always the same samples.
		//    Gives a fixed pattern in the shadow, but no noise
		//int index = i;
		//  - A random sample, based on the pixel's screen location. 
		//    No banding, but the shadow moves with the camera, which looks weird.
		//int index = int(16.0*random(gl_FragCoord.xyy, i))%16;
		//  - A random sample, based on the pixel's position in world space.
		//    The position is rounded to the millimeter to avoid too much aliasing
		int index = int(16.0*random(floor(position.xyz * 1000.0), i)) % 16;
		
		// being fully in the shadow will eat up 4*0.2 = 0.8
		// 0.2 potentially remain, which is quite dark.
		visibility -= 0.2 * (1.0 - texture(shadowMap, vec3(shadowCoords.xy + poissonDisk[index] / 700.0 , (shadowCoords.z - bias) / shadowCoords.w)));
	}
	visibility = max(visibility, 0.5f);
#endif
	
	return visibility;
}

#ifndef PCF
float ShadowCalculation(vec4 fragPosLightSpace)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadowMap, projCoords.xy).z;
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // check whether current frag pos is in shadow
    float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

	return shadow;
}
#endif

void main(){
	vec3 position = fs_in.p;
	vec3 normal = fs_in.n;
	// all vectors are in world space!
	vec3 N = flip * normalize ( normal);
	vec3 L = normalize ( light_position - position); // the light direction with respect to this point
	vec3 V = normalize ( eye_position - position);

	float ambient_light = 0.2; // TODO: uniform
	float cosTheta = max(dot(L, N), 0.0f);
	float diffuse_light = cosTheta;
	float specular_light = 0;

	// Compute specular component
	if(diffuse_light > 0 && length(material_ks) > 0){
		specular_light = pow(max(dot(V,reflect(-L,N)),0), material_shininess);	//Phong
		//specular_light = pow(max(dot(N,normalize(L+V)),0), material_shininess);		//Blinn - Phong
	}

	float shade = ((flags & OREN_NAYAR) != 0) ? orennayar(1.4) : (ambient_light + diffuse_light) / (1 + ambient_light);

	vec3 albedo = ((flags & TEXTURED) == 0) ? material_kd : material_kd * texture(map_color, fs_in.uv).xyz;

	vec3 mat_col = ((flags & VERTEX_COLORS) != 0) ? albedo * (1 - color.r) + color : albedo;
	// TODO: clamp mat_col
	//vec3 mat_col = albedo * (1 - color.r) + color;
	vec3 light = mat_col * shade + material_ks * specular_light;

	float visibility = 1.0f;
	if ((flags & SHADOWS) != 0)
		visibility = CalculateShadow(cosTheta);
	
	light = ((flags & MONOCHROME) != 0) ? vec3(material_kd) : vec3(visibility * light); // TODO: bypass all calculations for monochrome

	if ((flags & WIREFRAME) != 0)
	{
		// Wireframe rendering is better like this:
		vec3 dist_vec = dist * gl_FragCoord.w;
 
		// Compute the shortest distance to the edge
		float d = min(dist_vec[0], min(dist_vec[1], dist_vec[2]));
  
		// Cull fragments too far from the edge.
		const float LineWidth = 1.0;
		if (d > 0.5 * LineWidth + 1) d = 2;
	
		// Map the computed distance to the [0,2] range on the border of the line.
		d = clamp((d - (0.5*LineWidth - 1)), 0, 2);	
 
		// Compute line intensity and then fragment color
		float I = exp2(-2.0*d*d);
	
		// TODO: uniform for line color
		light = I * vec3(0, 0, 0) + (1.0 - I) * light;
	}
	
	out_color = vec4(light, 1);
}