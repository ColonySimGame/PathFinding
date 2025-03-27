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
}