using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using PathFindingGrid.Grid;
using PathFindingGrid.PathFinding;

namespace PathFindingGrid;

public abstract class Program
{
    public AStarSearch? AStar;
    
    [GlobalSetup]
    public void Setup()
    {
        const int width = 250;
        const int height = 250;
        const int depth = 10; // 3 Z-Levels (0, 1, 2)
        
        var map = new Map(new Tile[width, height, depth]);
        
        // Initialize map (example: all walkable)
        for (var z = 0; z < depth; z++)
        for (var y = 0; y < height; y++)
        for (var x = 0; x < width; x++)
            map.Tiles[x, y, z] = new Tile(new Point3D(x, y, z));

        // Add some obstacles
        map.Tiles[5, 5, 0].Walkable = false;
        map.Tiles[5, 6, 0].Walkable = false;
        map.Tiles[5, 7, 0].Walkable = false;
        map.Tiles[4, 6, 1].Walkable = false;
        map.Tiles[5, 6, 1].Walkable = false;
        map.Tiles[6, 6, 1].Walkable = false;
        
        // A wall from the top to the bottom with a hole at the top on z-level 1 on x = 5
        for (var y = 0; y < height; y++)
        {
            if (y != 0)
            {
                map.Tiles[10, y, 1].Walkable = false;
            }
        }

        // --- 4. Define Teleporters ---
        var rawTeleporters = new List<Teleporter>()
        {
            new(new Point3D(1, 1, 0), new Point3D(8, 8, 1), 2.0f),
            new(new Point3D(30, 8, 2), new Point3D(9, 10, 2), 0.5f),
            new(new Point3D(0, 0, 0), new Point3D(0, 0, 1), 1.0f),
            new(new Point3D(36, 13, 1), new Point3D(36, 13, 2), 1.0f),
            new(new Point3D(0, 15, 2), new Point3D(0, 15, 3), 1.0f),
            new(new Point3D(5, 8, 4), new Point3D(5, 8, 3), 1.0f),
        };
        foreach (var rawTeleporter in rawTeleporters)
        {
            map.AddTeleporter(rawTeleporter);
        }
        
        AStar = new AStarSearch(width, height, depth, map, allowDiagonal: true);
    }
    
    [Benchmark]
    public void AStarBenchmark()
    {
        // --- Define Start and Goal ---
        var start = new Point3D(0, 0, 0);
        var goal = new Point3D(9, 9, 4);

        // --- Find Path ---
        AStar?.FindPath(start, goal);
    }
    
    public static void Main(string[] args)
    {
        // Start benchmarking
        BenchmarkRunner.Run<Program>();
    }
    
}