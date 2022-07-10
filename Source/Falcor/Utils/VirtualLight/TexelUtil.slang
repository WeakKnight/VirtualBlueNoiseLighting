struct TexelUtil
{
	uint w;
	uint h;
	float texelW;
	float texelH;

	[mutating] void setup(uint2 dimension)
	{
		w = dimension.x;
		h = dimension.y;

		texelW = 1.0 / w;
		texelH = 1.0 / h;
	}

	/*
	e.g.
	|   |   |   |   |
	0  25  50  75   1
	*/
	float2 pixelSampleToUV(uint2 pixels)
	{
		return float2(texelW * (0.5 + pixels.x), texelH * (0.5 + pixels.y));
	}

	float2 pixelSampleToUVFloat(float2 pixels)
	{
		return float2(texelW * (0.5 + pixels.x), texelH * (0.5 + pixels.y));
	}
};
