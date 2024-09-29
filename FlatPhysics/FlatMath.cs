
namespace FlatPhysics
{
    public static class FlatMath // do everything we need to do with vectors
    {

        public static readonly float VerySmallAmount = 0.0005f; //0.5mm
        public static int Clamp(int min, int max, int value)
        {
            if (min == max)
            {
                return min;
            }
            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than max");
            }
            if (value < min)
            {
                return min;
            }
            if (value > max)
            {
                return max;
            }
            return value;
        }
        public static float Clamp(float min, float max, float value)
        {
            if (min == max) 
            {
                return min; 
            }
            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than max");
            }
            if (value < min)
            {
                return min;
            }
            if (value > max)
            {
                return max;
            }
            return value;
        }
        public static float LengthSquared(FlatVector v)
        {
            return (v.X * v.X + v.Y * v.Y);
        }
        public static float DistanceSquared(FlatVector a, FlatVector b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return (dx * dx + dy * dy);
        }
        public static float Length(FlatVector v)
        {
            return MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        }
        public static float Distance(FlatVector a, FlatVector b) 
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return MathF.Sqrt(dx * dx + dy * dy);
        }
        public static FlatVector Normalize(FlatVector v) 
        {
            return new FlatVector(v.X / Length(v), v.Y / Length(v));
        }
        public static float Dot(FlatVector a, FlatVector b) 
        {
            return a.X * b.X + a.Y * b.Y;
        }
        public static float Cross(FlatVector a, FlatVector b) 
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static bool NearlyEqual(float a, float b)
        {
            return MathF.Abs(a - b) < FlatMath.VerySmallAmount;
        }
        public static bool NearlyEqual(FlatVector a, FlatVector b)
        {
            return FlatMath.DistanceSquared(a, b) < FlatMath.VerySmallAmount* FlatMath.VerySmallAmount;    
        }
    }

}
