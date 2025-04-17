using PathFindingGrid.PathFinding;

namespace PathFindingGrid.Grid;

public class Map
{
    public Map(Tile[,,] tiles)
    {
        Tiles = tiles;
    }

    public Tile[,,] Tiles { get; set; }

    public List<Teleporter> Teleporters { get; set; } = [];
    public Dictionary<Point3D, List<Teleporter>> TeleportersByPoint { get; set; } = [];
    
    public void AddTeleporter(Teleporter teleporter)
    {
        Teleporters.Add(teleporter);
        
        TeleportersByPoint.TryAdd(teleporter.PointA, []);
        TeleportersByPoint[teleporter.PointA].Add(teleporter);
        
        TeleportersByPoint.TryAdd(teleporter.PointB, []);
        TeleportersByPoint[teleporter.PointB].Add(teleporter);
    }

    public Tile GetTile(Point3D point3D)
    {
        if (point3D.X < 0 || point3D.Y < 0 || point3D.Z < 0 ||
            point3D.X >= Tiles.GetLength(0) || point3D.Y >= Tiles.GetLength(1) || point3D.Z >= Tiles.GetLength(2))
        {
            throw new ArgumentOutOfRangeException(nameof(point3D), "Point is out of bounds");
        }

        return Tiles[point3D.X, point3D.Y, point3D.Z];
    }
}