using Flat;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework;
using System;
using System.Diagnostics.Contracts;

namespace FlatPhysics
{
    public sealed class FlatWorld
    {
        public static readonly float MinBodySize = 0.01f * 0.01f; // 1cm * 1cm
        public static readonly float MaxBodySize = 64f * 64f; // 64m * 64m

        public static readonly float MinDensity = 0.1f;// g/cm^3
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        private FlatVector gravity;
        private List<FlatBody> bodylist;
        private List<(int, int)> contactPairs;

        private FlatVector[] contactList;
        private FlatVector[] impulseList;
        private FlatVector[] raList;
        private FlatVector[] rbList;
        private FlatVector[] FrictionImpulseList;
        private float[] jList;

        public int BodyCount
        {
            get { return this.bodylist.Count; }
        }

        public FlatWorld()
        {
            this.gravity = new FlatVector(0f, -9.81f);
            this.bodylist = new List<FlatBody>();
            this.contactPairs = new List<(int, int)>();

            this.contactList = new FlatVector[2];
            this.impulseList = new FlatVector[2];
            this.raList = new FlatVector[2];
            this.rbList = new FlatVector[2];
            this.FrictionImpulseList = new FlatVector[2];
            this.jList = new float[2];
        }
        public void AddBody(FlatBody body)
        {
            this.bodylist.Add(body);
        }

        public bool RemoveBody(FlatBody body)
        {
            return this.bodylist.Remove(body);
        }

        public bool GetBody(int index, out FlatBody body)
        {
            body = null;

            if (index < 0 || index >= this.bodylist.Count) 
            {
                return false;
            }
            body = this.bodylist[index];
            return true;
            
        }

        public void Step(float time, int TotalIterations) 
        {
            TotalIterations = FlatMath.Clamp(FlatWorld.MinIterations, FlatWorld.MaxIterations, TotalIterations);

            for (int currentIteration = 0; currentIteration < TotalIterations; currentIteration++)
            {
                this.contactPairs.Clear();
                this.StepBodies(time, TotalIterations);
                this.BroadPhase();
                this.NarrowPhase();
                
            }
        }

        private void BroadPhase() 
        {
            // collision step .. Broad phase
            // loop over the bodies and check for intersection
            for (int i = 0; i < this.bodylist.Count - 1; i++)
            {
                FlatBody bodyA = this.bodylist[i];
                FlatAABB bodyA_aabb = bodyA.GetAABB();

                for (int j = i + 1; j < this.bodylist.Count; j++)
                {
                    FlatBody bodyB = this.bodylist[j];
                    FlatAABB bodyB_aabb = bodyB.GetAABB();

                    // don't resolve collision between two static bodies
                    if (bodyA.IsStatic && bodyB.IsStatic)
                    {
                        continue;
                    }

                    if (!Collision.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                    {
                        continue;
                    }
                    this.contactPairs.Add((i,j));
                }

            }
        }
        private void NarrowPhase() 
        {
            for (int i = 0; i < this.contactPairs.Count; i++)
            {
                (int, int) pair = this.contactPairs[i];
                FlatBody bodyA = this.bodylist[pair.Item1];
                FlatBody bodyB = this.bodylist[pair.Item2];

                if (Collision.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
                {
                    this.SeparateBodies(bodyA, bodyB, normal * depth); //minimum translation vector mtv
                    Collision.FindContactPoints(bodyA, bodyB, out FlatVector contact1, out FlatVector contact2, out int contactCount);
                    FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                    this.ResolveCollisionWithRotationAndFriction(in contact);
                }

            }
        }

        private void StepBodies(float time, int TotalIterations) 
        {
            // moving step
            for (int i = 0; i < this.bodylist.Count; i++)
            {
                this.bodylist[i].Step(time, this.gravity, TotalIterations);
            }

        }

        private void SeparateBodies(FlatBody bodyA, FlatBody bodyB, FlatVector mtv) 
        {

            if (bodyA.IsStatic)
            {
                bodyB.Move(mtv);

            }
            else if (bodyB.IsStatic)
            {
                bodyA.Move(-mtv);
            }
            else
            {
                bodyA.Move(-mtv / 2f);
                bodyB.Move(mtv / 2f);
            }

        }

        // without rotation or friction
        public void ResolveCollisionBasic(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal= contact.Normal;
            float depth = contact.depth;

            FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (FlatMath.Dot(relativeVelocity, normal) > 0f) 
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal); // e The ratio of final velocity to the initial velocity between two objects after their collision
            j /= (bodyA.InvMass + bodyB.InvMass);

            FlatVector impulse = j * normal;

        }
        public void ResolveCollisionWithRotation(in FlatManifold contact) 
        { 
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            FlatVector contact1 = contact.Contact1;
            FlatVector contact2 = contact.Contact2;
            int contactCount = contact.ContactCount;

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++) 
            {
                this.impulseList[i] = FlatVector.Zero;
                this.raList[i] = FlatVector.Zero;
                this.rbList[i] = FlatVector.Zero;
            }

            for (int i =0; i < contactCount; i++) 
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity = 
                    (bodyB.LinearVelocity + angularLinearVelocityB) - 
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = FlatMath.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = FlatMath.Dot(raPerp, normal);
                float rbPerpDotN = FlatMath.Dot(rbPerp, normal);    

                float denom = (bodyA.InvMass + bodyB.InvMass) + 
                    (raPerpDotN* raPerpDotN) * bodyA.InvInertia + 
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag; // e The ratio of final velocity to the initial velocity between two objects after their collision
                j /= denom;
                j /= (float)contactCount;

                FlatVector impulse = j * normal;
                impulseList[i] = impulse;

            }

            for (int i = 0; i < contactCount; i++) 
            {
                FlatVector impulse = impulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, impulse) * bodyA.InvInertia   ;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, impulse) * bodyB.InvInertia;
            }
        }
        public void ResolveCollisionWithRotationAndFriction(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            FlatVector contact1 = contact.Contact1;
            FlatVector contact2 = contact.Contact2;
            int contactCount = contact.ContactCount;

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float sf = (bodyA.StaticFriction + bodyB.StaticFriction)*0.5f;
            float df = (bodyA.DynamicFriction + bodyB.DynamicFriction)* 0.5f;

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = FlatVector.Zero;
                this.raList[i] = FlatVector.Zero;
                this.rbList[i] = FlatVector.Zero;
                this.FrictionImpulseList[i] = FlatVector.Zero;
                this.jList[i] = 0f;    
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = FlatMath.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = FlatMath.Dot(raPerp, normal);
                float rbPerpDotN = FlatMath.Dot(rbPerp, normal);

                float denom = (bodyA.InvMass + bodyB.InvMass) +
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia +
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag; // e The ratio of final velocity to the initial velocity between two objects after their collision
                j /= denom;
                j /= (float)contactCount;
                this.jList[i] = j;  

                FlatVector impulse = j * normal;
                impulseList[i] = impulse;

            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector impulse = impulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, impulse) * bodyB.InvInertia;
            }
                

            // Friction Impulse Calculations
            for (int i = 0; i < contactCount; i++)
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                FlatVector tangent = relativeVelocity - FlatMath.Dot(relativeVelocity, normal) * normal;

                if (FlatMath.NearlyEqual(tangent, FlatVector.Zero))
                {
                    continue;
                }
                else 
                {
                    tangent = FlatMath.Normalize(tangent);
                }

                float raPerpDotT = FlatMath.Dot(raPerp, tangent);
                float rbPerpDotT = FlatMath.Dot(rbPerp, tangent);

                float denom = (bodyA.InvMass + bodyB.InvMass) +
                    (raPerpDotT * raPerpDotT) * bodyA.InvInertia +
                    (rbPerpDotT * rbPerpDotT) * bodyB.InvInertia;

                float jt = -FlatMath.Dot(relativeVelocity, tangent); // e The ratio of final velocity to the initial velocity between two objects after their collision
                jt /= denom;
                jt /= (float)contactCount;

                FlatVector FrictionImpulse;
                float j = this.jList[i];

                if (MathF.Abs(jt) <= j * sf)
                {
                    FrictionImpulse = jt * tangent;
                }
                else 
                {
                    FrictionImpulse = -j * df * tangent;
                }

                 
                this.FrictionImpulseList[i] = FrictionImpulse;  
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector FrictionImpulse = this.FrictionImpulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -FrictionImpulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, FrictionImpulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += FrictionImpulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, FrictionImpulse) * bodyB.InvInertia;
            }
        }
    }
}
