import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.spark.SparkConf;
import org.apache.spark.api.java.JavaDoubleRDD;
import org.apache.spark.api.java.JavaRDD;
import org.apache.spark.api.java.JavaSparkContext;
import org.apache.spark.mllib.linalg.BLAS;
import org.apache.spark.mllib.linalg.Vector;
import org.apache.spark.mllib.linalg.Vectors;
import scala.Tuple2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import static java.lang.Math.sqrt;

public class G06HM4
{
    public static void main(String[] args) throws Exception
    {

        //------- PARSING CMD LINE ------------
        // Parameters are:
        // <path to file>, k, L and iter

        if (args.length != 4) {
            System.err.println("USAGE: <filepath> k L iter");
            System.exit(1);
        }
        String inputPath = args[0];
        int k=0, L=0, iter=0;
        try
        {
            k = Integer.parseInt(args[1]);
            L = Integer.parseInt(args[2]);
            iter = Integer.parseInt(args[3]);
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
        if(k<=2 && L<=1 && iter <= 0)
        {
            System.err.println("Something wrong here...!");
            System.exit(1);
        }
        //------------------------------------
        final int k_fin = k;

        //------- DISABLE LOG MESSAGES
        Logger.getLogger("org").setLevel(Level.OFF);
        Logger.getLogger("akka").setLevel(Level.OFF);

        //------- SETTING THE SPARK CONTEXT      
        SparkConf conf = new SparkConf(true).setAppName("kmedian new approach");
        JavaSparkContext sc = new JavaSparkContext(conf);

        //------- PARSING INPUT FILE ------------
        JavaRDD<Vector> pointset = sc.textFile(args[0], L)
                .map(x-> strToVector(x))
                .repartition(L)
                .cache();
        long N = pointset.count();
        System.out.println("Number of points is : " + N);
        System.out.println("Number of clusters is : " + k);
        System.out.println("Number of parts is : " + L);
        System.out.println("Number of iterations is : " + iter);

        //------- SOLVING THE PROBLEM ------------
        double obj = MR_kmedian(pointset, k, L, iter);
        System.out.println("Objective function is : <" + obj + ">");
    }

    public static Double MR_kmedian(JavaRDD<Vector> pointset, int k, int L, int iter)
    {
        //
        // --- ADD INSTRUCTIONS TO TAKE AND PRINT TIMES OF ROUNDS 1, 2 and 3 
        //

        long startR1 = System.currentTimeMillis();

        //------------- ROUND 1 ---------------------------
        JavaRDD<Tuple2<Vector,Long>> coreset = pointset.mapPartitions(x ->
        {
            ArrayList<Vector> points = new ArrayList<>();
            ArrayList<Long> weights = new ArrayList<>();
            while (x.hasNext())
            {
                points.add(x.next());
                weights.add(1L);
            }
            ArrayList<Vector> centers = kmeansPP(points, weights, k, iter);
            ArrayList<Long> weight_centers = compute_weights(points, centers);
            ArrayList<Tuple2<Vector,Long>> c_w = new ArrayList<>();
            for(int i =0; i < centers.size(); ++i)
            {
                Tuple2<Vector, Long> entry = new Tuple2<>(centers.get(i), weight_centers.get(i));
                c_w.add(i,entry);
            }
            return c_w.iterator();
        }).cache();

        // call the count() method to get the exact time to process the entire input and cache() method in order
        // to avoid to recompute all during the second round
        coreset.count();

        long endR1 = System.currentTimeMillis();

        //------------- ROUND 2 ---------------------------

        long startR2 = System.currentTimeMillis();

        ArrayList<Tuple2<Vector, Long>> elems = new ArrayList<>(k*L);
        elems.addAll(coreset.collect());
        ArrayList<Vector> coresetPoints = new ArrayList<>();
        ArrayList<Long> weights = new ArrayList<>();
        for(int i =0; i< elems.size(); ++i)
        {
            coresetPoints.add(i, elems.get(i)._1);
            weights.add(i, elems.get(i)._2);
        }

        ArrayList<Vector> centers = kmeansPP(coresetPoints, weights, k, iter);

        long endR2 = System.currentTimeMillis();

        //------------- ROUND 3: COMPUTE OBJ FUNCTION --------------------

        long startR3 = System.currentTimeMillis();

        double objectiveFunction = pointset.mapPartitionsToDouble(x -> {

            double distance;
            // distances from points to its closest center
            ArrayList<Double> dist = new ArrayList<>();
            // useful index to scroll through all the points of the dataset
            int i=0;
            Vector temp;

            while (x.hasNext())
            {
                temp = x.next();
                // initialize with distance between point i and the first center
                dist.add(sqrt(Vectors.sqdist(temp, centers.get(0))));
                // iterate each center
                for(int j=1; j<centers.size(); j++) {
                    // for each remaining center compute the distance between the center and the point i
                    distance = sqrt(Vectors.sqdist(temp, centers.get(j)));
                    // if the new computed distance is less than the previously distance
                    // update the minimum distance of point i
                    if (distance < dist.get(i)) dist.set(i,distance);
                }
                i++; // next point
            }

            return dist.iterator();

        }).mean(); // compute average of distances

        long endR3 = System.currentTimeMillis();

        // show the times of the three rounds
        System.out.println("Time for computing Round 1: " + (endR1 - startR1) + " ms");
        System.out.println("Time for computing Round 2: " + (endR2 - startR2) + " ms");
        System.out.println("Time for computing Round 3: " + (endR3 - startR3) + " ms");

        return objectiveFunction;
    }

    public static ArrayList<Long> compute_weights(ArrayList<Vector> points, ArrayList<Vector> centers)
    {
        Long weights[] = new Long[centers.size()];
        Arrays.fill(weights, 0L);
        for(int i =0; i < points.size(); ++i)
        {
            double tmp = euclidean(points.get(i), centers.get(0));
            int mycenter = 0;
            for(int j = 1; j < centers.size(); ++j)
            {
                if(euclidean(points.get(i),centers.get(j)) < tmp)
                {
                    mycenter = j;
                    tmp = euclidean(points.get(i), centers.get(j));
                }
            }
            weights[mycenter] += 1L;
        }
        ArrayList<Long> fin_weights = new ArrayList<>(Arrays.asList(weights));
        return fin_weights;
    }

    public static Vector strToVector(String str) {
        String[] tokens = str.split(" ");
        double[] data = new double[tokens.length];
        for (int i = 0; i < tokens.length; i++) {
            data[i] = Double.parseDouble(tokens[i]);
        }
        return Vectors.dense(data);
    }

    // Euclidean distance
    public static double euclidean(Vector a, Vector b) {
        return sqrt(Vectors.sqdist(a, b));
    }

    // method that return the closest center for each point of P
    public static ArrayList<Integer> Partition(ArrayList<Vector> P, ArrayList<Vector> centers) {

        // array of the same length of P, containing the corresponding centroid where the point is associated
        ArrayList<Integer> partition = new ArrayList<>();

        double distance;
        // for each point, 'dist' contain the distance between the point and closest center
        double dist;
        // for each point, 'center' contain the index of closest center
        int center;

        for (int p = 0; p < P.size(); p++) {
            // initialize with distance between point i and the first center
            dist = sqrt(Vectors.sqdist(P.get(p), centers.get(0)));
            // initialize with index of first center
            center = 0;
            // iterate each remaining center
            for (int c = 1; c < centers.size(); c++) {
                // for each remaining center compute the distance between the center and the point i
                distance = sqrt(Vectors.sqdist(P.get(p), centers.get(c)));
                // if the new computed distance is less than the previously distance
                // update the minimum distance and save the index of new closest center
                if(distance < dist)
                {
                    // new closest distance
                    dist = distance;
                    // new closest center
                    center = c;
                }
            }
            // add the closest center of point p
            partition.add(center);
        }

        return partition;
    }

    // method that compute the weighted objective function
    public static double computeObjectiveFunction(ArrayList<Vector> P, ArrayList<Long> WP, ArrayList<Integer> partition, ArrayList<Vector> centroids)  {

        double cost=0;

        for(int c=0;c<centroids.size();c++) {
            for(int p=0;p<P.size();p++) {
                // sum of the distance of each point to its cluster center multiplied for its weight
                if(partition.get(p)==c) cost += WP.get(p) * sqrt(Vectors.sqdist(P.get(p), centroids.get(c)));
            }
        }

        return cost;
    }

    // method that return k clustering centers of the points provided in input applying first a variant of kmeans++
    // algorithm and next refine the centers applying 'iter' iterations of Lloyds' algorithm. Since we use k-median
    // the objective function can get worse during the iterations, therefore we consider the minimum value of the
    // objective function calculated in the 'iter' iterations.
    public static ArrayList<Vector> kmeansPP(ArrayList<Vector> P, ArrayList<Long> WP, int k, int iter) {

        // S, set of centers of kmeans++ algorithm
        ArrayList<Vector> centers = new ArrayList<>();
        // array useful for saving the weights eliminated in kmeans++
        ArrayList<Long> saveWP = new ArrayList<>();

        Random r = new Random();
        // choose a random point and set it as a first center
        int c1 = r.nextInt(P.size());
        centers.add(P.get(c1));
        // remove this point from datapoint P
        P.remove(c1);
        // save the weight of point for reuse it in Lloyds
        saveWP.add(WP.get(c1));
        // remove the corresponding weight
        WP.remove(c1);

        // vector of minimum distances between each point and its closest center
        ArrayList<Double> dist = new ArrayList<>();

        // compute the remaining k-1 centers
        for (int m = 0; m < k-1; m++) {
            // vector of probability that each point has for become a center
            ArrayList<Double> probabilities = new ArrayList<>();
            double sum = 0;

            // compute the sum of minimum distances for each point in P-S
            for (int i = 0; i < P.size(); i++) {
                // in the first iteration save the distances between all points and the first center
                if(m == 0) dist.add(sqrt(Vectors.sqdist(P.get(i), centers.get(m))));
                else {
                    // compute the distance between each point and the new computed center
                    double distance = sqrt(Vectors.sqdist(P.get(i), centers.get(m)));
                    // update the closest distance between points in P-S and new added centers
                    // update a distance only if is less than the previous one, this guarantees O(|P|*k)
                    if(distance < dist.get(i)) {
                        dist.set(i,distance);
                    }
                }
                // sum the distances for computing probabilities
                sum += dist.get(i) * WP.get(i);
            }

            // probabilities for each Point in P-S
            for (int i = 0; i < P.size(); i++) {
                // compute the probability that a point has for become a center
                probabilities.add((dist.get(i) * WP.get(i)) / sum);
            }

            // choose a random point from 0 to 1
            double choice = Math.random();

            double var = 0;
            int index = 0;

            // in order to choose a next point that become a center we sum the probabilities
            // until the sum is greater than the random number compute earlier. Then we pick
            // the point that its probability contain the random point
            for (int i = 0; i < probabilities.size(); i++) {
                if (var < choice) {
                    var += probabilities.get(i);
                    index = i;
                }
            }

            // we add the next center as the point that its probability contain the random point
            centers.add(P.get(index));
            // remove the point from P
            P.remove(index);
            // save the weights for reuse them in Lloyds
            saveWP.add(WP.get(index));
            // remove weight of point
            WP.remove(index);
            // remove the distance of the point just eliminated from P
            dist.remove(index);

        }

        // once the C centers have been found, refine them with Lloyds' algorithm

        // if there aren't iterations, return centers computed with kmeans++
        if(iter == 0) return centers;

        // re-enter the points in P and weights in WP
        for(int i=0;i<centers.size();i++) {
            P.add(centers.get(i));
            WP.add(saveWP.get(i));
        }

        // set of centroids C', may not be part of points P
        ArrayList<Vector> centroids = new ArrayList<>();
        // best centroids found
        ArrayList<Vector> bestCentroids = new ArrayList<>();
        // vector that contain for each point its closest center
        ArrayList<Integer> partition;

        double objectiveFunction;
        // initialize the value of objective function
        double bestObjectiveFunction = Double.MAX_VALUE;

        // max "iter" iterations of Lloyd algorithm
        for (int l = 0; l < iter; l++) {

            // find the partition of centers computed with kmeans++
            if(l == 0) partition = Partition(P, centers);
                // find for each point its closest centroid
            else partition = Partition(P, centroids);

            centroids = new ArrayList<>();

            // adjust each centroid
            for (int m = 0; m < k; m++) {

                int sumW = 0;
                // use a temporary variable for compute a new centroid
                Vector temp;
                double[] data = new double[P.get(0).size()];
                for (int i = 0; i < P.get(0).size(); i++) {
                    data[i] = 0;
                }
                temp = Vectors.dense(data);

                for (int i = 0; i < P.size(); i++) {
                    // choose only point associated with centroid m
                    if (partition.get(i) == m) {
                        // sum p*w(p) of points belonging to the centroid m
                        BLAS.axpy(WP.get(i), P.get(i), temp);
                        // sum of the weights of the points belonging to the centroid m
                        sumW += WP.get(i);
                    }
                }
                // number of points associated with centroid m
                double N = 1.0 / sumW;
                // divide the summation for the number of points
                BLAS.scal(N, temp);
                // add the new computed centroid
                centroids.add(temp);

            }

            // objective function computed with the new centroids
            objectiveFunction = computeObjectiveFunction(P,WP,partition,centroids);
            // find the minimum of objective function
            if(objectiveFunction < bestObjectiveFunction) {
                // update the best objective function
                bestObjectiveFunction = objectiveFunction;
                // update the best centroids
                bestCentroids = centroids;
            }

        }

        // return the centroids
        return bestCentroids;

    }
}
