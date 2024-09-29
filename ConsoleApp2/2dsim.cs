using System;
using System.Diagnostics;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Audio;
using Flat;
using Flat.Graphics;
using Flat.Input;
using FlatPhysics;
using System.Drawing;
using System.Numerics;
using FlatMath = FlatPhysics.FlatMath;
using ConsoleApp2;

namespace FlatAsteroids
{
    public class Game1 : Game
    {
        private readonly GraphicsDeviceManager graphics;
        private Screen screen;
        private Sprites sprites;
        private Shapes shapes;
        private Camera camera;
        private SpriteFont fontConsolas18;

        private FlatWorld world;

        private List<FlatEntity> entityList;
        private List<FlatEntity> entityRemoveList;

        private Stopwatch watch;

        private double totalWorldStepTime = 0d;
        private int totalBodyCount = 0;
        private int totalSampleCount = 0;
        private Stopwatch sampleTimer = new Stopwatch();

        private string worldStepTimeString = String.Empty;
        private string bodyCountString = String.Empty;

        public Game1()
        {
            this.graphics = new GraphicsDeviceManager(this);
            this.graphics.SynchronizeWithVerticalRetrace = true;

            this.Content.RootDirectory = "Content";
            this.IsMouseVisible = true;
            this.IsFixedTimeStep = true;

            const double UpdatesPerSecond = 60d;
            this.TargetElapsedTime = TimeSpan.FromTicks((long)Math.Round((double)TimeSpan.TicksPerSecond / UpdatesPerSecond));
        }


        protected override void Initialize()
        {
            this.Window.Position = new Microsoft.Xna.Framework.Point(10, 40);

            DisplayMode displaymode = GraphicsAdapter.DefaultAdapter.CurrentDisplayMode;
            int height = displaymode.Height;
            int width = displaymode.Width;
            this.graphics.PreferredBackBufferHeight = height;
            this.graphics.PreferredBackBufferWidth = width;
            this.graphics.ApplyChanges();


            this.screen = new Screen(this, 1280, 768);
            this.sprites = new Sprites(this);
            this.shapes = new Shapes(this);
            this.camera = new Camera(this.screen);
            this.camera.Zoom = 20;

            this.camera.GetExtents(out float left, out float right, out float bottom, out float top);

            this.entityList = new List<FlatEntity>();
            this.entityRemoveList = new List<FlatEntity>();

            this.world = new FlatWorld();

            float padding = MathF.Abs(right - left) * 0.10f;

            if (!FlatBody.CreateRectanguleBody(right - left - padding * 2, 3f, 1f,
                0.5f, true, out FlatBody groundBody, out string errorMessage)) 
            {
                throw new Exception(errorMessage);
            }
                
            groundBody.MoveTo(new FlatVector(0, -10));
            this.world.AddBody(groundBody);
            this.entityList.Add(new FlatEntity(groundBody, Microsoft.Xna.Framework.Color.DarkGray));


            if (!FlatBody.CreateRectanguleBody(20f, 2f, 1f,  0.5f, true, out FlatBody ledgeBody1, out errorMessage)) 
            {
                throw new Exception(errorMessage);
            }
            ledgeBody1.MoveTo(new FlatVector(-10, 3));
            ledgeBody1.Rotate(-MathHelper.TwoPi/20f);
            this.world.AddBody(ledgeBody1);
            this.entityList.Add(new FlatEntity(ledgeBody1 ,Microsoft.Xna.Framework.Color.DarkGray));

            if (!FlatBody.CreateRectanguleBody(15f, 2f, 1f, 0.5f, true, out FlatBody ledgeBody2, out errorMessage))
            {
                throw new Exception(errorMessage);
            }

            ledgeBody2.MoveTo(new FlatVector(10, 10));
            ledgeBody2.Rotate(MathHelper.TwoPi/20f);
            this.world.AddBody(ledgeBody2);
            this.entityList.Add(new FlatEntity(ledgeBody2, Microsoft.Xna.Framework.Color.DarkGray));

            this.watch = new Stopwatch();
           
            base.Initialize();
        }

        protected override void LoadContent()
        {
        }

        protected override void Update(GameTime gameTime)
        {

            FlatKeyboard keyboard = FlatKeyboard.Instance;
            FlatMouse mouse = FlatMouse.Instance;

            keyboard.Update();
            mouse.Update();

            // add box 
            if (mouse.IsLeftMouseButtonPressed())
            {
                float width = RandomHelper.RandomSingle(2f, 3f);
                float height = RandomHelper.RandomSingle(2f, 3f);

                FlatVector mouseWorldPostion =
                    FlatConverter.ToFlatVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));

                this.entityList.Add(new FlatEntity(this.world, width, height, false, mouseWorldPostion));
            }

            // add circle
            if (mouse.IsRightMouseButtonPressed())
            {
                float raduis = RandomHelper.RandomSingle(1f, 1.25f);

                FlatVector mouseWorldPostion =
                    FlatConverter.ToFlatVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));

                this.entityList.Add(new FlatEntity(this.world, raduis, false, mouseWorldPostion));
            }

            if (keyboard.IsKeyAvailable)
            {
                if (keyboard.IsKeyClicked(Keys.P))
                {
                    Console.WriteLine($"Body count: {this.world.BodyCount}");
                    Console.WriteLine($"Step time: {Math.Round(this.watch.Elapsed.TotalMilliseconds, 4)} ms");
                    Console.WriteLine();
                }
                if (keyboard.IsKeyClicked(Keys.Escape))

                {
                    this.Exit();
                }

                if (keyboard.IsKeyClicked(Keys.A))
                {
                    this.camera.IncZoom();
                }

                if (keyboard.IsKeyClicked(Keys.Z))
                {
                    this.camera.DecZoom();
                }
            }

            this.watch.Restart();
            this.world.Step(FlatUtil.GetElapsedTimeInSeconds(gameTime), 20);
            this.watch.Stop();

            this.totalWorldStepTime += this.watch.Elapsed.TotalMilliseconds;
            this.totalBodyCount += this.world.BodyCount;
            this.totalSampleCount++;



            this.camera.GetExtents(out _, out _, out float viewbottom, out _);

            this.entityRemoveList.Clear();

            //remove bodies that are out of the view except static bodies
            for (int i = 0; i < this.entityList.Count; i++)
            {
                FlatEntity entity = this.entityList[i];
                FlatBody body = entity.Body;

                if (body.IsStatic)
                {
                    continue;
                }
                FlatAABB box = body.GetAABB();

                if (box.Max.Y < viewbottom)
                {
                    this.entityRemoveList.Add(entity);
                }
            }
            for (int i = 0; i < this.entityRemoveList.Count; i++) 
            {
                FlatEntity entity = this.entityRemoveList[i];
                this.world.RemoveBody(entity.Body);
                this.entityList.Remove(entity);
            }

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            this.screen.Set();
            this.GraphicsDevice.Clear(new Microsoft.Xna.Framework.Color(50, 60, 70));


            this.shapes.Begin(this.camera);
            for (int i = 0; i < this.entityList.Count; i++)
            {
                this.entityList[i].Draw(this.shapes);
            }
            //List<FlatVector> contactPoints = this.world?.ContactPointsList;
            //for (int i =0; i< contactPoints.Count; i++) 
            //{
            //    Microsoft.Xna.Framework.Vector2 contactPoint =  FlatConverter.ToVector2(contactPoints[i]);
            //    shapes.DrawBoxFill(contactPoint, 0.3f, 0.3f, Microsoft.Xna.Framework.Color.Red);
            //    shapes.DrawBox(contactPoint, 0.3f, 0.3f, Microsoft.Xna.Framework.Color.White);
            //}
            
            this.shapes.End();

            this.screen.Unset();
            this.screen.Present(this.sprites);

            base.Draw(gameTime);
        }
    }
}
