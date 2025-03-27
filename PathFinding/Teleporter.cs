namespace PathFindingGrid.PathFinding;

public struct Teleporter
{
    public Point3D PointA;
    public Point3D PointB;
    public float Cost;

    public Teleporter(Point3D pointA, Point3D pointB, float cost)
    {
        PointA = pointA;
        PointB = pointB;
        Cost = cost;
    }
}