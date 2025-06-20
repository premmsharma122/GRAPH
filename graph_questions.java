import java.util.*;

public class graph_questions {
    public static class Edge {
        int src;
        int dest;
        int wt;

        public Edge(int s, int d , int w){
            this.src = s;
            this.dest = d;
            this.wt = w;
        }
    }

    public static void createGraph(ArrayList<Edge>[] graph, int flights[][]){
        for(int i=0; i<graph.length; i++){
            graph[i] = new ArrayList<>();
        }
        for(int i=0; i<flights.length; i++){
            int src = flights[i][0];
            int dest = flights[i][1];
            int wt = flights[i][2];
            graph[src].add(new Edge(src, dest, wt));
        }
    }

    static class Info {
        int v;     // vertex
        int cost;
        int stops;

        public Info(int v, int c , int s){
            this.v = v;
            this.cost = c;
            this.stops = s;
        }
    }

    public static int cheapestFlights(int n, int src, int dest , int k, int flights[][]){
        ArrayList<Edge>[] graph = new ArrayList[n];
        createGraph(graph, flights);

        int[] dist = new int[n];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[src] = 0;

        Queue<Info> q = new LinkedList<>();
        q.add(new Info(src, 0, 0));  // start from source

        while(!q.isEmpty()){
            Info curr = q.remove();
            if(curr.stops > k) continue;

            for(Edge e : graph[curr.v]){
                int u = e.src;
                int v = e.dest;
                int wt = e.wt;

                if(curr.cost + wt < dist[v]){
                    dist[v] = curr.cost + wt;
                    q.add(new Info(v, dist[v], curr.stops + 1));
                }
            }
        }

        return dist[dest] == Integer.MAX_VALUE ? -1 : dist[dest];
    }
    // Question 2 : Connecting all cities with lowest cost By PRIMS'S ALGORITHM.
    static class Edge2 implements Comparable<Edge2>{
        int dest;
        int cost;

        public Edge2(int d , int c){
            this.dest = d;
            this.cost = c;
        }
        @Override
        public int compareTo(Edge2 e2){
            return this.cost - e2.cost; // ascending order sorting
        }

    }
    public static int connectCities(int cities[][]){
        PriorityQueue<Edge2> pq  = new PriorityQueue<>();
        boolean seen[] = new boolean[cities.length];

        pq.add(new Edge2(0, 0));
        int finalCost = 0;

        while(!pq.isEmpty()){
            Edge2 curr = pq.remove();
            if(!seen[curr]){
                seen[curr]= true;
                finalCost += curr.cost;

                for(int i=0; i<cities[curr.dest].length; i++){
                    if(cities[curr.dest][i] != 0){
                        pq.add(new Edge2(i,cities[curr.dest][i] ))
                    }
                }
            }
        }

        return finalCost;
    }

    public static void main(String args[]){
        int n = 5;
        int flights[][] = {
            {0, 1, 100}, 
            {1, 2, 100}, 
            {2, 0, 100},
            {1, 3, 600}, 
            {2, 3, 200}
        };
        int src = 0, dest = 3, k = 1;
        System.out.println(cheapestFlights(n, src, dest, k, flights)); // Output should be 700
    }
}
