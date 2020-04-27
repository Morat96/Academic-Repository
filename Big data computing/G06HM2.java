import org.apache.spark.SparkConf;
import org.apache.spark.api.java.JavaDoubleRDD;
import org.apache.spark.api.java.JavaPairRDD;
import org.apache.spark.api.java.JavaRDD;
import org.apache.spark.api.java.JavaSparkContext;
import scala.Tuple2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ThreadLocalRandom;


public class G06HM2 {

    // Improved Word count 1 algorithm
    public static JavaPairRDD<String, Long> wordCount1(JavaRDD<String> docs) {

        long start = System.currentTimeMillis();

        JavaPairRDD<String, Long> wordcountpairs = docs
                // Map phase
                .flatMapToPair((document) -> {
                    String[] tokens = document.split(" ");
                    HashMap<String, Long> counts = new HashMap<>();
                    ArrayList<Tuple2<String, Long>> pairs = new ArrayList<>();
                    for (String token : tokens) {
                        counts.put(token, 1L + counts.getOrDefault(token, 0L));
                    }
                    for (Map.Entry<String, Long> e : counts.entrySet()) {
                        pairs.add(new Tuple2<>(e.getKey(), e.getValue()));
                    }
                    return pairs.iterator();
                })
                // Reduce phase
                .reduceByKey((x,y)->x+y);

        // call the count method to get the exact time to process the entire input
        wordcountpairs.count();

        // time for computing Word Count 1
        long end = System.currentTimeMillis();
        System.out.println("Time for computing Word Count 1: " + (end - start) + " ms");

        return wordcountpairs;
    }

    // Improved Word count 2 algorithm : First variant
    public static JavaPairRDD<String, Long> wordCount2v1(JavaRDD<String> docs, long k) {

        long start = System.currentTimeMillis();

        JavaPairRDD<String, Long> wordcountpairs = docs
                // ROUND 1
                // Map phase
                .flatMapToPair((document) -> {
                    String[] tokens = document.split(" ");
                    HashMap<String, Long> counts = new HashMap<>();
                    ArrayList<Tuple2<String, Long>> pairs = new ArrayList<>();
                    for (String token : tokens) {
                        counts.put(token, 1L + counts.getOrDefault(token, 0L));
                    }
                    for (Map.Entry<String, Long> e : counts.entrySet()) {
                        pairs.add(new Tuple2<>(e.getKey(), e.getValue()));
                    }
                    return pairs.iterator();
                })
                // assign to each key-value pair a random key in [0,k-1]
                .groupBy(x -> ThreadLocalRandom.current().nextLong(k))
                // Reduce phase
                .flatMapToPair((it) -> {
                    // function that for each word-count pair with the same key sum the
                    // occurrences with the same word
                    HashMap<String, Long> counts = new HashMap<>();
                    ArrayList<Tuple2<String, Long>> pairs = new ArrayList<>();
                    for (Tuple2<String,Long> c : it._2()) {
                        counts.put(c._1, c._2 + counts.getOrDefault(c._1, 0L));
                    }
                    for (Map.Entry<String, Long> e : counts.entrySet()) {
                        pairs.add(new Tuple2<>(e.getKey(), e.getValue()));
                    }
                    return pairs.iterator();
                })
                // ROUND 2
                // Map Phase : Identity
                // Reduce Phase
                .reduceByKey((x,y)-> x+y);

        // call the count method to get the exact time to process the entire input
        wordcountpairs.count();

        // time for computing Word Count 2: First Variant
        long end = System.currentTimeMillis();
        System.out.println("Time for computing Word Count 2 - First variant: " + (end - start) + " ms");

        return wordcountpairs;
    }

    // Improved Word count 2 algorithm : Second variant
    public static JavaPairRDD<String, Long> wordCount2v2(JavaRDD<String> docs) {

        long start = System.currentTimeMillis();

        JavaPairRDD<String, Long> wordcountpairs = docs
                // mapPartitionsToPair works on the k partitions of JavaRDD docs therefore the algorithm
                // uses a single phase that computes the key-value pairs in each partition
                // Map phase
                .mapPartitionsToPair((document) -> {
                    // function that for each partition, split the documents
                    // and sum the occurrences with the same word
                    HashMap<String, Long> counts = new HashMap<>();
                    ArrayList<Tuple2<String, Long>> pairs = new ArrayList<>();
                    while(document.hasNext()) {
                        String[] tokens = document.next().split(" ");
                        for (String token : tokens) {
                            counts.put(token, 1L + counts.getOrDefault(token, 0L));
                        }
                    }
                    for (Map.Entry<String, Long> e : counts.entrySet()) {
                        pairs.add(new Tuple2<>(e.getKey(), e.getValue()));
                    }

                    return pairs.iterator();

                },true)
                // Reduce phase
                .reduceByKey((x,y)->x+y);

        // call the count method to get the exact time to process the entire input
        wordcountpairs.count();

        // time for computing Word Count 2: Second Variant
        long end = System.currentTimeMillis();
        System.out.println("Time for computing Word Count 2 - Second variant: " + (end - start) + " ms");

        return wordcountpairs;
    }

    // method for compute the average length of the distinct words appearing in the documents
    public static double AVGwordsLength(JavaPairRDD<String, Long> result) {

        // Apply a function, to each String element of JavaPairRDD result, that for every word return its
        // length, next store the result in a JavaDoubleRDD
        JavaDoubleRDD len = result.mapToDouble((x)->(double) x._1.length());

        // compute the average of lengths with method mean of JavaDoubleRDD interface
        double average = len.mean();

        return average;

    }

    public static void main(String[] args) throws IOException {
        if (args.length == 0) {
            throw new IllegalArgumentException("Expecting the file name on the command line");
        }

        SparkConf conf =
                new SparkConf(true)
                        .setAppName("HM2");

        JavaSparkContext sc = new JavaSparkContext(conf);

        // PART 1

        // input arguments example : 10 text-sample.txt

        // number of partitions provided in input
        int k = Integer.parseInt(args[0]);

        // RDD representing the documents
        JavaRDD<String> docs = sc.textFile(args[1]).cache();
        // call count method for forcing the caching
        docs.count();
        // subdivide the JavaRDD docs in k partitions
        docs.repartition(k).cache();


        // PART 2

        // call Improved Word count 1 method
        JavaPairRDD<String, Long> wordCount1 = wordCount1(docs);

        // call Improved Word count 2 - First variant method
        JavaPairRDD<String, Long> wordCount2 = wordCount2v1(docs,k);

        // call Improved Word count 2 - Second variant method
        JavaPairRDD<String, Long> wordCount3 = wordCount2v2(docs);

        // PART 3

        // function for compute the average length of the distinct words appearing in the documents
        double average = AVGwordsLength(wordCount1);

        // print of average length of the distinct words appearing in the documents
        System.out.println("The average length of the distinct words is " + average);

        // print of all distinct words and their lengths
        System.out.println("These are all words with respective number of occurrences in entire collection");
        System.out.println(wordCount3.collect());


    }

}
