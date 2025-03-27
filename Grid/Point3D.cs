namespace PathFindingGrid.Grid;

public readonly struct Point3D : IEquatable<Point3D>
{
    public int X { get; }
    public int Y { get; }
    public int Z { get; }

    public Point3D(int x, int y, int z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public bool Equals(Point3D other)
    {
        return X == other.X && Y == other.Y && Z == other.Z;
    }

    public override bool Equals(object? obj)
    {
        return obj is Point3D other && Equals(other);
    }

    public override int GetHashCode()
    {
        // Simple hash combining coordinates
        return HashCode.Combine(X, Y, Z);
    }

    public override string ToString()
    {
        return $"({X}, {Y}, {Z})";
    }

    public static bool operator ==(Point3D left, Point3D right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(Point3D left, Point3D right)
    {
        return !left.Equals(right);
    }
}