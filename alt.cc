// pick landmarks on the maps, perhaps 10 points
// precompute distances from landmarks to all the other vertices
// distance from v to t
// d(v, t) = d(A, t) - d(A, v),
// d(v, t) = d(v, A) - d(t, A)
// d(v, t) >= max(d(A, t) - d(A, v), d(v, A) - d(t, A))