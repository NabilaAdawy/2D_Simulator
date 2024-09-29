using System;

namespace FlatPhysics
{
    public static class Collision
    {
        public static void PointSegmentDistance(FlatVector p, FlatVector a, FlatVector b, out float distanceSquared, out FlatVector cp) 
        {
            FlatVector ab = b - a;
            FlatVector ap = p - a;

            float proj = FlatMath.Dot(ap, ab);
            float abLenSq = FlatMath.LengthSquared(ab);
            float d = proj / abLenSq;

            if (d <= 0f) 
            {
                cp = a; 
            }
            else if (d >= 1f)
            {
                cp = b; 
            }
            else
            {
                cp = a + ab * d; 
            }

            distanceSquared = FlatMath.DistanceSquared(p, cp);
        }
        public static bool IntersectAABBs(FlatAABB a, FlatAABB b) 
        {
            if (a.Max.X <= b.Min.X || b.Max.X <= a.Min.X ||
                a.Max.Y <= b.Min.Y || b.Max.Y <= a.Min.Y) 
            {
                return false;
            }

            return true;
        }
        public static void FindContactPoints(
            FlatBody bodyA, FlatBody bodyB, 
            out FlatVector contact1, out FlatVector contact2, 
            out int contactCount) 
        {
            contact1 = FlatVector.Zero; 
            contact2 = FlatVector.Zero; 
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Rectangule)
            {
                if (shapeTypeB is ShapeType.Rectangule)
                {
                    Collision.FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), out contact1, out contact2, out contactCount);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collision.FindCirclePolygonContactPoint(bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
            }

            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Rectangule)
                {
                    Collision.FindCirclePolygonContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collision.FindCirclesContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, out contact1);
                    contactCount = 1;
                }
            }
        }

        public static void FindPolygonsContactPoints(
            FlatVector[] verticesA, FlatVector[] verticesB,
            out FlatVector contact1, out FlatVector contact2, out int contactCount) 
        {
            contact1 = FlatVector.Zero;
            contact2 = FlatVector.Zero;
            contactCount = 0;

            float minDistSq = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++) 
            {
                FlatVector p = verticesA[i];
                for (int j = 0; j < verticesB.Length; j++)
                {
                    FlatVector va = verticesB[j];
                    FlatVector vb = verticesB[(j + 1) % verticesB.Length];

                    Collision.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if (distSq == minDistSq) 
                    { 
                        if (!FlatMath.NearlyEqual(cp, contact1)) //not already equal to the first contact point
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if (distSq < minDistSq) 
                    { 
                        minDistSq = distSq; 
                        contact1 = cp; 
                        contactCount = 1;
                    }

                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector p = verticesB[i];
                for (int j = 0; j < verticesA.Length; j++)
                {
                    FlatVector va = verticesA[j];
                    FlatVector vb = verticesA[(j + 1) % verticesA.Length];

                    Collision.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if (distSq == minDistSq)
                    {
                        if (!FlatMath.NearlyEqual(cp, contact1)) //not already equal to the first contact point
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contact1 = cp;
                        contactCount = 1;
                    }

                }
            }
        }
        private static void FindCirclePolygonContactPoint(
            FlatVector circleCenter, float circleRaduis, 
            FlatVector polygonCenter, FlatVector[] polygonVertices, 
            out FlatVector cp) 
        {
            cp = FlatVector.Zero;
            float minDistSq = float.MaxValue;

            for (int i = 0; i < polygonVertices.Length; i++) 
            {
                FlatVector va = polygonVertices[i];
                FlatVector vb = polygonVertices[(i + 1) % polygonVertices.Length];

                Collision.PointSegmentDistance(circleCenter, va, vb, out float distSq, out FlatVector contact); //contact is the candidate of cp 

                if (distSq < minDistSq) 
                {
                    minDistSq = distSq;
                    cp = contact;
                }

            }
        }
        private static void FindCirclesContactPoint(FlatVector centerA, float radiusA, FlatVector centerB, out FlatVector cp)
        {
            FlatVector ab = centerB - centerA;
            FlatVector dir = FlatMath.Normalize(ab);
            cp = centerA + dir * radiusA;

        }
        public static bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Rectangule)
            {
                if (shapeTypeB is ShapeType.Rectangule)
                {
                    return Collision.IntersectPolygons(
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);

                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    bool results = Collision.IntersectCirclePolygon(
                        bodyB.Position, bodyB.Radius,
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        out normal, out depth);

                    normal = -normal;
                    return results;
                }
            }

            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Rectangule)
                {
                    return Collision.IntersectCirclePolygon(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);

                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    return Collision.IntersectCircles(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.Radius,
                        out normal, out depth);
                }
            }
            return false;
        }

        public static bool IntersectCirclePolygon(FlatVector circleCenter, float circleRaduis,
            FlatVector polyCenter, FlatVector[] vertices, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;

            FlatVector axis = FlatVector.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < vertices.Length; i++)
            {
                FlatVector va = vertices[i];
                FlatVector vb = vertices[(i + 1) % vertices.Length];

                FlatVector edge = vb - va;

                axis = new FlatVector(-edge.Y, edge.X); //rotated to be perpendicular to the edge & its magnitude is = 'edge'
                axis = FlatMath.Normalize(axis); // to get the same scale for each loop

                Collision.ProjectVertices(vertices, axis, out minA, out maxA);
                Collision.ProjectCircle(circleCenter, circleRaduis, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            int closestPointIndex = Collision.FindClosestPointOnPolygon(circleCenter, vertices);
            FlatVector cp = vertices[closestPointIndex];

            axis = cp - circleCenter;
            axis = FlatMath.Normalize(axis);

            Collision.ProjectVertices(vertices, axis, out minA, out maxA);
            Collision.ProjectCircle(circleCenter, circleRaduis, axis, out minB, out maxB);

            if (minA >= maxB || minB >= maxA)
            {
                return false;
            }

            axisDepth = MathF.Min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }

            FlatVector centerDiretion = polyCenter - circleCenter;

            if (FlatMath.Dot(centerDiretion, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }
        private static int FindClosestPointOnPolygon(FlatVector circleCenter, FlatVector[] vertices) 
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < vertices.Length; i++) 
            {
                FlatVector v = vertices[i]; 
                float distance = FlatMath.Distance(v, circleCenter);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    result = i;
                }
            }
            return result;
        }
        public static bool IntersectPolygons(FlatVector centerA, FlatVector[] verticesA, FlatVector centerB, FlatVector[] verticesB, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++)
            {
                FlatVector va = verticesA[i];
                FlatVector vb = verticesA[(i + 1) % verticesA.Length];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X); //rotated to be perpendicular to the edge & its magnitude is = 'edge'
                axis = FlatMath.Normalize(axis); // to get the same scale for each loop

                Collision.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collision.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }
                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector va = verticesB[i];
                FlatVector vb = verticesB[(i + 1) % verticesB.Length];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X);
                axis = FlatMath.Normalize(axis);

                Collision.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collision.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }
                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            FlatVector centerDiretion = centerB - centerA;

            if (FlatMath.Dot(centerDiretion, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }
        private static void ProjectCircle(FlatVector center, float raduis, FlatVector axis, out float min, out float max) 
        { 
            FlatVector direction = FlatMath.Normalize(axis);
            FlatVector directionAndRaduis = direction * raduis;

            FlatVector pA = center + directionAndRaduis;
            FlatVector pB = center - directionAndRaduis;

            min = FlatMath.Dot(pA, axis);
            max = FlatMath.Dot(pB, axis);

            if (min > max) 
            { 
                float t = min; 
                min = max; 
                max = t; 
            }
        }
        private static void ProjectVertices(FlatVector[] vertices, FlatVector axis, out float min, out float max) 
        {
            min = float.MaxValue;
            max = float.MinValue;

            for (int i = 0; i < vertices.Length; i++)
            {
                FlatVector va = vertices[i];
                float proj = FlatMath.Dot(va, axis);

                if (proj < min) { min = proj; } 
                if (proj > max) { max = proj; }
            }
        }

        public static bool IntersectCircles(FlatVector centerA, float raduisA, FlatVector centerB, float raduisB,out FlatVector normal, out float depth) 
        {
            normal = FlatVector.Zero;
            depth = 0f;

            float distance = FlatMath.Distance(centerA, centerB);
            float radii = raduisA + raduisB;

            if ( distance >= radii)
            {
                return false;       
            }
            normal = FlatMath.Normalize(centerB - centerA);
            depth = radii - distance;
            return true;
        }
    }
}
