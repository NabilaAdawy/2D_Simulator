using Flat;
using Flat.Graphics;
using FlatPhysics;
using System;
using System.Drawing;
using System.Numerics;

namespace ConsoleApp2
{
    public sealed class FlatEntity
    {
        public FlatBody Body;
        public readonly Microsoft.Xna.Framework.Color Color;

        public FlatEntity(FlatBody body)
        {
            this.Body = body;
            this.Color = RandomHelper.RandomColor();
        }
        public FlatEntity(FlatBody body, Microsoft.Xna.Framework.Color color)
        {
            this.Body = body;
            this.Color = color;
        }
        public FlatEntity(FlatWorld world, float radius, bool isStatic, FlatVector position) 
        {
            if (!FlatBody.CreateCircleBody(radius, 1f, 0.5f, isStatic, out FlatBody body, out string errorMessage))
            {
                throw new Exception(errorMessage);
            }
            body.MoveTo(position);
            this.Body = body;
            world.AddBody(this.Body);
            this.Color = RandomHelper.RandomColor();
        }

        public FlatEntity(FlatWorld world, float width, float height, bool isStatic, FlatVector position)
        {
            if (!FlatBody.CreateRectanguleBody(width, height, 1f, 0.5f, isStatic, out FlatBody body, out string errorMessage))
            {
                throw new Exception(errorMessage);
            }
            body.MoveTo(position);
            this.Body = body;
            world.AddBody(this.Body);
            this.Color = RandomHelper.RandomColor();
        }

        public void Draw(Shapes shapes) 
        {
            Microsoft.Xna.Framework.Vector2 position = FlatConverter.ToVector2(this.Body.Position);

            if (this.Body.ShapeType is ShapeType.Circle)
            {
                Microsoft.Xna.Framework.Vector2 va = Vector2.Zero;
                Microsoft.Xna.Framework.Vector2 vb = new Vector2(Body.Radius, 0f);
                Flat.FlatTransform Transform = new Flat.FlatTransform(position, Body.Angle, 1);
                va = Flat.FlatUtil.Transform(va, Transform);
                vb = Flat.FlatUtil.Transform(vb, Transform);
                    
                shapes.DrawCircleFill(position, this.Body.Radius, 26, this.Color);
                shapes.DrawCircle(position, this.Body.Radius, 26, Microsoft.Xna.Framework.Color.White);
                shapes.DrawLine(va, vb, Microsoft.Xna.Framework.Color.White);
            }
            else if (this.Body.ShapeType is ShapeType.Rectangule) 
            {

                shapes.DrawBoxFill(position, this.Body.Width, this.Body.Height, this.Body.Angle, this.Color);
                shapes.DrawBox(position, this.Body.Width, this.Body.Height, this.Body.Angle, Microsoft.Xna.Framework.Color.White);
            }
        }
    }
}
