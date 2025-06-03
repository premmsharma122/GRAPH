import java.util.*;

public class graph {
    public static class Edge {
        int src;
        int dest;
        int wt;
        public Edge(int s, int d, int w) {
            this.src = s;
            this.dest = d;
            this.wt = w;
        }
    }
    // Components : It help to travers in different part of a graph/ tree
    public static void bfs(ArrayList<Edge>[] graph){
        boolean seen[] = new boolean[graph.length];
        for(int i=0; i<graph.length; i++){
            if(!seen[i]){
                bfsUtils(graph, seen);
            }
        }
    }

    // Breadth First Search
    public static void bfsUtils(ArrayList<Edge>[] graph, boolean seen[]){
        Queue<Integer> q = new LinkedList<>();
        
        q.add(0); // add starting node
        while(!q.isEmpty()){
            int curr = q.remove();
            if(!seen[curr]){
                System.out.print(curr+" ");
                seen[curr] = true;
                for(int i = 0; i < graph[curr].size(); i++){
                    Edge e = graph[curr].get(i);
                    q.add(e.dest);
                }
            }
        }
    }
    //componet for dfs
    public static void dfs(ArrayList<Edge>[] graph ){
        boolean seen[] = new boolean[graph.length];
        for(int i=0; i<graph.length; i++){
            if(!seen[i]){
                dfsUtils(graph, i, seen);
            }
        }
    }
    // Depth first search
    public static void dfsUtils(ArrayList<Edge>[] graph , int curr, boolean seen[]){
        System.out.print(curr+" ");
        seen[curr] = true;
        for(int i=0; i<graph[curr].size(); i++){
            Edge e = graph[curr].get(i);
            if(!seen[e.dest]){
                dfsUtils(graph, e.dest, seen);
            }
        }
    }
    // is path or Not
    public static boolean isPath(ArrayList<Edge>[] graph, int src , int trg, boolean seen[]){
        if(src == trg) return true;
        seen[src] = true;
        for(int i=0; i<graph[src].size(); i++){
            Edge e = graph[src].get(i);
            if(!seen[e.dest] && isPath(graph, e.dest, trg,seen)){
                return true;
            }
        }
        return false;
    }
    //Detect Cycle
    public static boolean detectCycle(ArrayList<Edge>[] graph){
        boolean seen[] = new boolean[graph.length];
        for(int i=0; i<graph.length; i++){
            if(detectCycleUtils(graph, seen , i, -1)){
                return true;
            }
        }
        return false;
    }
    public static boolean detectCycleUtils(ArrayList<Edge>[] graph,boolean[]  seen, int curr, int par){
        seen[curr] = true;

        for(int i=0; i<graph[curr].size(); i++){
            Edge e = graph[curr].get(i);
            //case 1 : if neigbour is not visited but detect function return it true then it is cycle
            if(!seen[e.dest] && detectCycleUtils(graph, seen, e.dest, curr)){
                return true;
            }
            //case 2 : if visited and not a parent then : return true
            if(seen[e.dest] && par != e.dest){
                return true;
            }
            // case 3 : Do nothing -> continue
        }
        return false;

    }
    public static boolean isBipartite(ArrayList<Edge>[] graph){
        int col[] = new int[graph.length];
        for(int i=0; i<col.length; i++){
            col[i] = -1;
        }
        Queue<Integer> q = new LinkedList<>();
        for(int i=0; i<graph.length; i++){
            q.add(i);
            col[i] = 0; // yellow
            while(!q.isEmpty()){
                int curr = q.remove();
                for(int j=0; j<graph[curr].size(); j++){
                    Edge e = graph[curr].get(j);
                    if(col[e.dest] == -1){
                        int nextCol = col[curr]==0 ? 1 : 0;
                        col[e.dest] = nextCol;
                        q.add(e.dest);
                    }else if(col[e.dest] == col[curr]){
                        return false;
                    }

                }
            }
        }
        return true;
    }
    public static boolean isCycleDirected(ArrayList<Edge>[] graph){
        boolean stack[] = new boolean[graph.length];
        boolean seen[] = new boolean[graph.length];

        for(int i=0; i<graph.length; i++){
            if(!seen[i]){
                if(isCycleDirectedUtils(graph, i, stack, seen)){
                    return true;
                }
            }
        }
        return false;
    }
    public static boolean isCycleDirectedUtils(ArrayList<Edge>[] graph, int curr, boolean stack[], boolean seen[]){
        stack[curr] = true;
        seen[curr] = true;

        for(int i=0; i<graph.length; i++){
            Edge e = graph[curr].get(i);
            if(stack[e.dest]){
                return true;
            } 
            else if(!seen[e.dest] && isCycleDirectedUtils(graph, e.dest, stack, seen)){
                return true;
            }
        }
        stack[curr] =false;
        return false;
    }
    public static void topSort(ArrayList<Edge>[] graph){
        Stack<Integer> st = new Stack<>();
        boolean seen[] = new boolean[graph.length];

        for(int i=0; i<graph.length; i++){
            if(!seen[i]){
                topSortUtils(graph, i , seen, st);
            }
        }
        while(!st.isEmpty()){
            System.out.print(st.pop()+" ");
        }
    }
    public static void topSortUtils(ArrayList<Edge>[] graph, int curr, boolean seen[] , Stack<Integer> st){
        seen[curr] = true;

        for(int i=0; i<graph[curr].size(); i++){
            Edge e = graph[curr].get(i);
            if(!seen[e.dest]){
                topSortUtils(graph, e.dest, seen, st);
            }
        }
        st.push(curr);
    }

    public static void  calIndegre(ArrayList<Edge>[] graph, int arr[]){
        for(int i=0; i<graph.length; i++){
            int v = i;
            for(int j=0; j<graph[v].size(); j++){
                Edge e = graph[v].get(j);
                arr[e.dest]++;
            }
        }
    }

    public static void topSortBFS(ArrayList<Edge>[] graph){
        int inDeg[] = new int[graph.length];
        Queue<Integer> q = new LinkedList<>();
        calIndegre(graph, inDeg);

        for(int i=0; i<graph.length; i++){
            if(inDeg[i] == 0){
                q.add(i);
            }
        }
        //bfs
        while(!q.isEmpty()){
            int curr = q.remove();
            System.out.println(curr+" "); // print topological sort

            for(int i=0; i<graph[curr].size(); i++){
                Edge e = graph[curr].get(i);
                inDeg[e.dest]--;
                if(e.dest == 0){
                    q.add(e.dest);
                }
            }
        }
        System.out.println();
    }
    public static void printAllPaths(ArrayList<Edge>[] graph, int src ,int dest, String path){
        if(src == dest){
            System.out.println(path+dest);;
            return;
        }

        for(int i=0; i<graph[src].size(); i++){
            Edge e = graph[src].get(i);
            printAllPaths(graph, e.dest, dest, path+src);
        }
    }
    // Dijkstra Algorithm -> To find smallest part b/w source to destination.
    static class Pair implements Comparable<Pair>{
        int n;
        int path;
        public Pair(int n, int path){
            this.n = n;
            this.path = path;
        }
        @Override
        public int compareTo(Pair P2){
            return this.path - P2.path;  // path based sorting
        }
    }

    public static void dijkstra(ArrayList<Edge>[] graph, int src){
        int dist[] = new int[graph.length];
        for(int i=0; i<graph.length; i++){
            if( i!= src){
                dist[i] = Integer.MAX_VALUE;
            }
        }
        boolean seen[] = new boolean[graph.length];
        PriorityQueue<Pair> pq = new PriorityQueue<>();
        pq.add(new Pair(src, 0));
        while(!pq.isEmpty()){
            Pair curr = pq.remove();
            if(!seen[curr.n]){
                seen[curr.n]= true;
                //neighbours
                for(int i=0; i<graph[curr.n].size(); i++){
                    Edge  e = graph[curr.n].get(i);
                    int u = e.src;
                    int v = e.dest;
                    int wt = e.wt;
                    if(dist[u] + wt < dist[v]){
                        dist[v] = dist[u] + wt;
                        pq.add(new Pair(v, dist[v]));
                    }
                }
            }
        }
        for(int i=0; i<dist.length; i++){
            System.out.print(dist[i]+" ");
        }
        System.out.println();
    }
    public static void bellmanFord(ArrayList<Edge>[] graph, int src){
        int dist[] = new int[graph.length];
        for(int i=0; i<dist.length; i++){
            if(i != src){
                dist[i] = Integer.MAX_VALUE;
            }
        }
        int V = graph.length;
        for(int i=0; i<V-1; i++){
            //edges O(E)
            for(int j=0; j<graph.length; j++){
                for(int k=0; k<graph[j].size(); k++){
                    Edge e = graph[j].get(k);
                    int u = e.src;
                    int v = e.dest;
                    int wt = e.wt;
                    // relation part
                    if(dist[u] != Integer.MAX_VALUE && dist[u] + wt < dist[v]){
                        dist[v] = dist[u] + wt;
                    }
                }

            }
        }
        for(int i=0; i<dist.length; i++){
            System.out.println(dist[i] + " ");
        }
    }
    // MST : Minimum Spanning Tree By Prims Algorithm ->
    static class MSTPair implements Comparable<MSTPair> {
        int v;
        int cost;

        public MSTPair(int v , int cost){
            this.v = v;
            this.cost = cost;
        }
        @Override
        public int compareTo(MSTPair p2){
            return this.cost - p2.cost;  // cost based sorting
        }
    }
    public static void prims(ArrayList<Edge>[] graph){
        boolean seen [ ] = new boolean[graph.length];
        PriorityQueue<MSTPair> pq = new PriorityQueue<>();

        pq.add(new MSTPair(0,0));
        int finalcost =0;
        while(!pq.isEmpty()){
            MSTPair curr = pq.remove();
            if(!seen[curr.v]){
                seen[curr.v] = true;
                finalcost += curr.cost;
                for(int i=0; i<graph[curr.v].size(); i++){
                    Edge e = graph[curr.v].get(i);
                    if (!seen[e.dest]) {  // ✅ To avoid adding already visited nodes
                        pq.add(new MSTPair(e.dest, e.wt));
                    }
                }
            }
        }
        System.out.println(finalcost);
    }
    

    public static void createGraph(ArrayList<Edge>[] graph){
        for(int i = 0; i < graph.length; i++){
            graph[i] = new ArrayList<>();
        }

        graph[0].add(new Edge(0, 1, 1));
        graph[0].add(new Edge(0, 2, 1));

        graph[1].add(new Edge(1, 0, 1));
        graph[1].add(new Edge(1, 3, 1));

        graph[2].add(new Edge(2, 0, 1));
        graph[2].add(new Edge(2, 4, 1));

        graph[3].add(new Edge(3, 1, 1));
        graph[3].add(new Edge(3, 4, 1));
        graph[3].add(new Edge(3, 5, 1));

        graph[4].add(new Edge(4, 2, 1));
        graph[4].add(new Edge(4, 3, 1));
        graph[4].add(new Edge(4, 5, 1));

        graph[5].add(new Edge(5, 3, 1));
        graph[5].add(new Edge(5, 4, 1));
        graph[5].add(new Edge(5, 6, 1));

        graph[6].add(new Edge(6, 5, 1));
    }

    public static void main(String[] args) {
        int V = 7; // total number of vertices
        ArrayList<Edge>[] graph = new ArrayList[V];
        createGraph(graph);
        // System.out.println("BFS traversal:");
        // bfs(graph);
        boolean seen[] = new boolean[V];
        // dfs(graph, 0, seen);
        // System.out.println(detectCycle(graph));
        // topSort(graph);
        printAllPaths(graph, 0, 5, null);
    }
}
