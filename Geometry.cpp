#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>
#define ll long long
#define int ll
#define ld long double
#define pb push_back

#define ff first
#define ss second
#define mp make_pair
#define files(in, out) freopen(in, "r", stdin); freopen(out, "w", stdout)
#define rep(i, n) for (int i = 0; i < n; ++i)
#define all(a) a.begin(), a.end()
#define pii pair<int, int
#define cin_vec(a) rep(i, n) cin >> a[i];

using namespace std;

const int max_n = 20;

int inf = 3e18;
int mod = 1e9 + 7;
ld eps = 1e-8;
const ld pi = acos(-1);

int gcd(int a, int b) {
    if (b == 0) return a;
    return gcd(b, a % b);
}

struct Point {
    ld x, y;
    Point() {}
    Point(ld x, ld y) : x(x), y(y) {}
    void read() {
        cin >> x;
        cin >> y;
    }
    bool operator == (Point p) {
        return x == p.x && y == p.y;
    }
};

struct Vector {
    ld x, y;
    Point beg, end;
    Vector() {}
    Vector(Point beg, Point end) {
        this->x = end.x - beg.x;
        this->y = end.y - beg.y;
        this->beg = beg;
        this->end = end;
    }
    Vector(ld x, ld y) : x(x), y(y) {}
    Vector operator * (long double b) {
        return Vector(x * b, y * b);
    }
    Vector operator / (long double b) {
        return Vector(x / b, y / b);
    }
    Vector operator + (Vector a) {
        return Vector(x + a.x, y + a.y);
    }
};

struct Line {
    ld a, b, c;
    Point p1, p2;
    Line() {}
    Line(Point p1, Point p2) {
        a = p1.y - p2.y;
        b = p2.x - p1.x;
        c = -a * p1.x - b * p1.y;
        this->p1 = p1;
        this->p2 = p2;
    }
};

struct Segment {
    Point a, b;
    Segment() {}
    Segment(Point a, Point b) : a(a), b(b) {}
};

struct Ray {
    Point a, b;
    Ray() {}
    Ray(Point a, Point b) : a(a), b(b) {}
};

struct Angle {
    Point p1, center, p2;
    Angle() {}
    Angle(Point p1, Point p2, Point p3) : p1(p1), center(p2), p2(p3) {}
};

struct Circle {
    Point p;
    double r;
};

//-------------------------------------------------------------------------------------
// functions

long double vectorLength(Vector vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
}

long double dotProduct(Vector a, Vector b) {
    return a.x * b.x + a.y * b.y;
}

long double crossProduct(Vector a, Vector b) {
    return a.x * b.y - a.y * b.x;
}

long double pointDistance(Point a, Point b) {
    return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
}

Vector normalize(Vector vec, long double len) {
    ld vec_len = vectorLength(vec);
    vec = vec * len;
    vec = vec / vec_len;
    return vec;
}

Vector normal(Vector vec) {
    long double a = vec.end.y - vec.beg.y;
    long double b = vec.beg.x - vec.end.x;
    return Vector(a, b);
}

long double distance(Line line, Point p) {
    Vector PP1 = Vector(p, line.p1), PP2 = Vector(p, line.p2), P1P2 = Vector(line.p1, line.p2);
    return abs(crossProduct(PP1, PP2)) / vectorLength(P1P2);
}

long double distance(Ray ray, Point p) {
    if (dotProduct(Vector(ray.a, p), Vector(ray.a, ray.b)) < 0) {
        return pointDistance(p, ray.a);
    }
    Vector PP1 = Vector(p, ray.a), PP2 = Vector(p, ray.b), P1P2 = Vector(ray.a, ray.b);
    return abs(crossProduct(PP1, PP2)) / vectorLength(P1P2);
}

long double distance(Segment seg, Point p) {
    return max(distance(Ray(seg.a, seg.b), p), distance(Ray(seg.b, seg.a), p));
}

bool isContained(Line line, Point a) {
    return (line.a * a.x + line.b * a.y + line.c == 0);
}

bool isContained(Segment seg, Point p) {
    if (!isContained(Line(seg.a, seg.b), p))
        return false;
    if (dotProduct(Vector(seg.a, p), Vector(seg.a, seg.b)) >= 0 && dotProduct(Vector(seg.b, seg.a), Vector(seg.b, p)) >= 0)
        return true;
    else
        return false;
}

Point intersect(Line line1, Line line2) {
    long double a1 = line1.a, b1 = line1.b, c1 = line1.c;
    long double a2 = line2.a, b2 = line2.b, c2 = line2.c;
    if ((a1 * b2 - a2 * b1) == 0) return Point(-INT_MAX, -INT_MAX);
    Point p;
    p.x = (c2 * b1 - c1 * b2) / (a1 * b2 - a2 * b1);
    p.y = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1);
    return p;
}

bool isIntersect(Segment seg1, Segment seg2) {
    if (max(seg1.a.x, seg1.b.x) >= min(seg2.a.x, seg2.b.x) &&
        max(seg2.a.x, seg2.b.x) >= min(seg1.a.x, seg1.b.x) &&
        max(seg1.a.y, seg1.b.y) >= min(seg2.a.y, seg2.b.y) &&
        max(seg2.a.y, seg2.b.y) >= min(seg1.a.y, seg1.b.y)) {
        Vector v1 = Vector(seg1.a, seg2.a);
        Vector v2 = Vector(seg1.a, seg1.b);
        Vector v3 = Vector(seg1.a, seg2.b);
        Vector v4 = Vector(seg2.a, seg1.a);
        Vector v5 = Vector(seg2.a, seg2.b);
        Vector v6 = Vector(seg2.a, seg1.b);
        if (crossProduct(v2, v1) * crossProduct(v2, v3) <= 0 && crossProduct(v5, v4) * crossProduct(v5, v6) <= 0) {
            return true;
        }
    }
    return false;
}

pair<Point, Point> intersect(Segment seg1, Segment seg2) {
    if (!isIntersect(seg1, seg2)) return { Point(-INT_MAX, -INT_MAX), Point(-INT_MAX, -INT_MAX) };
    if (seg1.a == seg1.b && seg2.a == seg2.b) {
        if (seg1.a == seg2.a)
            return { seg1.a, Point(-INT_MAX, -INT_MAX) };
    }
    if (seg1.a == seg1.b) {
        if (isContained(seg2, seg1.a)) {
            Point p1 = seg1.a;
            return { p1, Point(-INT_MAX, -INT_MAX) };
        }
    }
    if (seg2.a == seg2.b) {
        if (isContained(seg1, seg2.a)) {
            Point p1 = seg2.a;
            return { p1, Point(-INT_MAX, -INT_MAX) };
        }
    }
    if ((isContained(Line(seg1.a, seg1.b), seg2.a) && isContained(Line(seg1.a, seg1.b), seg2.b))) {
        vector<Point> points;
        points.pb(seg1.a);
        points.pb(seg1.b);
        points.pb(seg2.a);
        points.pb(seg2.b);
        sort(points.begin(), points.end(), [](Point a, Point b) {
            if (a.x == b.x)
                return a.y < b.y;
            return a.x < b.x;
            });
        Point p1, p2;
        p1.x = points[1].x;
        p1.y = points[1].y;
        if (!(points[1] == points[2])) {
            p2.x = points[2].x;
            p2.y = points[2].y;
        } else {
            p2 = Point(-INT_MAX, -INT_MAX);
        }
        return { p1, p2 };
    } else {
        Point p = intersect(Line(seg1.a, seg1.b), Line(seg2.a, seg2.b));
        if (p.x != -INT_MAX && p.y != -INT_MAX) {
            Point p1 = p;
            if (isContained(seg1, p) && isContained(seg2, p)) {
                return { p1, Point(-INT_MAX, -INT_MAX) };
            } else {
                return { Point(-INT_MAX, -INT_MAX), Point(-INT_MAX, -INT_MAX) };
            }
        }
    }
}

pair<Point, Point> Tangent(Circle circle, Point p) {
    long double p0p1 = pointDistance(circle.p, p);
    long double p2p1 = sqrt(p0p1 * p0p1 - circle.r * circle.r);
    long double p1p3 = p2p1 * p2p1 / p0p1;
    Vector p1p3_vec = normalize(Vector(p, circle.p), p1p3);
    Point p3;
    p3.x = p.x + p1p3_vec.x;
    p3.y = p.y + p1p3_vec.y;
    // first_point
    Vector norm = normal(Vector(p, p3));
    long double p3p2 = circle.r * sqrt(p0p1 * p0p1 - circle.r * circle.r) / p0p1;
    Vector p3p2_vec = normalize(norm, p3p2);
    Point p2;
    p2.x = p3.x + p3p2_vec.x;
    p2.y = p3.y + p3p2_vec.y;
    // second_point
    norm = normal(Vector(p, p3));
    norm = norm * (-1);
    p3p2 = circle.r * sqrt(p0p1 * p0p1 - circle.r * circle.r) / p0p1;
    p3p2_vec = normalize(norm, p3p2);
    Point p4;
    p4.x = p3.x + p3p2_vec.x;
    p4.y = p3.y + p3p2_vec.y;
    return { p2, p4 };
}

bool isContained(Circle circle, Point p) {
    return abs((p.x - circle.p.x) * (p.x - circle.p.x) + (p.y - circle.p.y) * (p.y - circle.p.y) - circle.r * circle.r) <= eps;
}

int isIntersect(Circle circle, Segment seg) {
    long double dist = distance(seg, circle.p);
    if (abs(distance(seg, circle.p) - circle.r) <= eps)
        return 1;
    else if (distance(seg, circle.p) > circle.r)
        return 0;
    else
        return 2;
}

int isIntersect(Circle circle1, Circle circle2) {
    double r1 = circle1.r;
    double r2 = circle2.r;
    double d = pointDistance(circle1.p, circle2.p);
    if (abs(r1 + r2 - d) <= eps || abs(abs(r1 - r2) - d) <= eps)
        return 1;
    else if (r1 + r2 < d || abs(r1 - r2) > d)
        return 0;
    else
        return 2;
}

long double circleDistance(Circle circle, Point p1, Point p2) {
    Vector pp1 = Vector(circle.p, p1);
    Vector pp2 = Vector(circle.p, p2);
    long double angle = atan2(crossProduct(pp1, pp2), dotProduct(pp1, pp2));
    if (angle < 0) angle += 2 * pi;
    return min((long double)angle * circle.r, (2 * pi - angle) * circle.r);
}

pair<Point, Point> intersect(Circle circle1, Circle circle2) {
    double r1 = circle1.r;
    double r2 = circle2.r;
    Point o1 = circle1.p;
    Point o2 = circle2.p;
    double l = pointDistance(o1, o2);
    double o1h = (-r2 * r2 + r1 * r1 + l * l) / (2 * l);
    Vector o1h_vec = normalize(Vector(o1, o2), o1h);
    Point h;
    h.x = o1.x + o1h_vec.x;
    h.y = o1.y + o1h_vec.y;
    double hp = sqrt(r1 * r1 - o1h * o1h);
    Vector norm1 = normal(Vector(o1, o2));
    Vector norm2 = normal(Vector(o1, o2)) * (-1);
    Vector hp_vec1 = normalize(norm1, hp);
    Vector hp_vec2 = normalize(norm2, hp);
    Point p1, p2;
    p1.x = h.x + hp_vec1.x;
    p1.y = h.y + hp_vec1.y;
    p2.x = h.x + hp_vec2.x;
    p2.y = h.y + hp_vec2.y;
    return { p1, p2 };
}

Line biss(Point p1, Point p2, Point p3) {
    Vector v(p2, p3);
    Vector v2(p2, p1);
    v2 = normalize(v2, vectorLength(v));
    p1 = Point(p2.x + v2.x, p2.y + v2.y);
    Point p4(p1.x + v.x, p1.y + v.y);
    Line line(p2, p4);
    return line;
}

Line perpend(Line line, Point p) {
    Vector vec(line.b, -line.a);
    Point p1 = Point(p.x + line.a, p.y + line.b);
    Line new_line = Line(p, p1);
    return new_line;
}

Point middlePoint(Point p1, Point p2) {
    return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}

ld angleVector(Vector v1, Vector v2) {
    ld ans = atan2(abs(crossProduct(v1, v2)), dotProduct(v1, v2));
    if (ans < 0) {
        ans += 2 * pi;
    }
    return ans;
}

void solve() {

}

signed main() {
    files("input.txt", "output.txt");
    int t = 1;
    while (t--)
        solve();
    return 0;
}