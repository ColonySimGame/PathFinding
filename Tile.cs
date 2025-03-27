namespace PathFindingGrid;

public class Tile
{
    public Tile(Point3D point3D, bool walkable = true, int speedModifier = 100)
    {
        Point3D = point3D;
        Walkable = walkable;
        SpeedModifier = speedModifier;
    }

    public Point3D Point3D { get; }

    public bool Walkable { get; set; }
    public int SpeedModifier { get; set; }
    
    public Dictionary<(int x, int y, int z), int> Costs { get; set; } = [];
    
    public List<(int x, int y, int z, bool isDiag)> CanMoveTo { get; set; } = [];
    
    public override string ToString()
    {
        return Walkable ? "O" : "X";
    }
}