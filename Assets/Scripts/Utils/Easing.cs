public static class Easing
{
    public static float Linear(float t) { return t; }

    public static float EaseInQuad(float t) { return t * t; }

    public static float EaseOutQuad(float t) { return t * (2 - t); }

    public static float EaseInOutQuad(float t) { return t < .5f ? 2 * t * t : -1 + (4 - 2 * t) * t; }

    public static float EaseInCubic(float t) { return t * t * t; }

    public static float EaseOutCubic(float t) { return (--t) * t * t + 1; }

    public static float EaseInOutCubic(float t) { return t < .5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1; }

    public static float EaseInQuart(float t) { return t * t * t * t; }

    public static float EaseOutQuart(float t) { return 1 - (--t) * t * t * t; }

    public static float EaseInOutQuart(float t) { return t < .5 ? 8 * t * t * t * t : 1 - 8 * (--t) * t * t * t; }

    public static float EaseInQuint(float t) { return t * t * t * t * t; }

    public static float EaseOutQuint(float t) { return 1 + (--t) * t * t * t * t; }

    public static float EaseInOutQuint(float t) { return t < .5 ? 16 * t * t * t * t * t : 1 + 16 * (--t) * t * t * t * t; }
}