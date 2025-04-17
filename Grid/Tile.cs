namespace PathFindingGrid.Grid;

public class Tile
{
    public Tile(Point3D point3D, bool walkable = true, double speedModifier = 1.0)
    {
        Point3D = point3D;
        Walkable = walkable;
        SpeedModifier = speedModifier;
        
        // TODO: Remove
        // RAndomize the speed modifier between 0.5 and 1.3
        SpeedModifier = Random.Shared.NextDouble() * 0.8 + 0.5;
    }

    public Point3D Point3D { get; }

    public bool Walkable { get; set; }
    public double SpeedModifier { get; set; }
    
    public Dictionary<(int x, int y, int z), int> Costs { get; set; } = [];
    
    public List<(int x, int y, int z, bool isDiag)> CanMoveTo { get; set; } = [];
    
    public override string ToString()
    {
        return Walkable ? "O" : "X";
    }
}