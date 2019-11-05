package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.Callable;

import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Range;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;

public class MoveletsDiscoveryPivots implements Callable<Integer> {
	
	private List<ISubtrajectory> candidates;

	private ITrajectory trajectory;
	
	private List<ITrajectory> trajectories;

	private IDistanceMeasureForSubtrajectory dmbt;
	
	private IQualityMeasure qualityMeasure;
	
	private int minSize;
	
	private int maxSize;
	
	private boolean cache;
	
	private boolean exploreDimensions;
	
	private int maxNumberOfFeatures = 2;
	
	private Random random;
	
	private int limit_size = 0;
	
	private int porcentage;
	
	private Boolean only_pivots = false;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
			
	public MoveletsDiscoveryPivots(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions,
			int maxNumberOfFeatures, int porcentage, boolean only_pivots) {
		this.candidates = candidates;
		this.trajectory = trajectory;
		this.trajectories = trajectories;
		this.dmbt = dmbt;
		this.qualityMeasure = qualityMeasure;
		this.minSize = minSize;
		this.maxSize = maxSize;
		this.cache = cache;
		this.exploreDimensions = exploreDimensions;
		this.maxNumberOfFeatures = maxNumberOfFeatures;
		this.porcentage = porcentage;
		this.only_pivots = only_pivots;

		if (this.maxNumberOfFeatures < 1 || this.maxNumberOfFeatures > dmbt.getDMBP().getNumberOfFeatures())
			this.maxNumberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();

	}
	
	@Override
	public Integer call() throws Exception {
		
		measureShapeletCollection();
		
		return 0;
	}
	
	public void measureShapeletCollection() {
		
		List<ISubtrajectory> candidates;
		
		this.random = new Random(trajectory.getTid());
		
		//Change to the desirable approach
		//candidates = fastMoveletsDiscoveryUsingCacheFirstPivotApproach(trajectory, trajectories, minSize, maxSize);
		candidates = fastMoveletsDiscoveryUsingCacheSecondPivotApproach(trajectory, trajectories, minSize, maxSize, random);

		for (ISubtrajectory candidate : candidates) {
			/*
			 * STEP 1: COMPUTE DISTANCES, IF NOT COMPUTED YET
			 */
			if (candidate.getDistances() == null)		
				ComputeDistances(candidate);
			
			/*
			 * STEP 2: ASSES QUALITY, IF REQUIRED
			 */
			if (qualityMeasure != null & candidate.getQuality() != null)
				AssesQuality(candidate);
		}
		
		/* STEP 3: SELECTING BEST CANDIDATES
		 * */		
		this.candidates.addAll(MoveletsFilterAndRanker.getShapelets(candidates));
		//this.candidates.addAll(candidates);
	
	}
	
	private void AssesQuality(ISubtrajectory candidate) {
		qualityMeasure.assesQuality(candidate);
	}
	
	private void AssesQuality(ISubtrajectory candidate, Random random) {
		qualityMeasure.assesQuality(candidate, random);
	}
	
	private void ComputeDistances(ISubtrajectory candidate) {
		
		/* This pairs will store the subtrajectory of the best alignment 
		 * of the candidate into each trajectory and the distance 
		 * */
		Pair<ISubtrajectory,double[]> distance;
		
		double[][] trajectoryDistancesToCandidate = new double[candidate.getSplitpoints().length]
															  [trajectories.size()];
		
		ISubtrajectory[] bestAlignments = new ISubtrajectory[trajectories.size()];
				
		/* It calculates the distance of trajectories to the candidate
		 */
		for (int i = 0; i < trajectories.size(); i++) {
			
			distance = dmbt.getBestAlignment(candidate, trajectories.get(i));
						
			bestAlignments[i] = distance.getFirst();
			trajectoryDistancesToCandidate[i] = distance.getSecond();			
		}
		
		candidate.setDistances(trajectoryDistancesToCandidate);
		candidate.setBestAlignments(bestAlignments);
	}

	
	private List<ISubtrajectory> getCandidatesUsingMDist(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist) {
		
		int n = trajectory.getData().size();
				
		List<ISubtrajectory> candidates = new ArrayList<>();

		for (int start = 0; start <= (n - size); start++) {

			candidates.addAll(getCandidatesFromTraj(trajectory, train, size, start, mdist));

		} // for (int start = 0; start <= (n - size); start++)
		
		return candidates;
		
	}
	
	private List<ISubtrajectory> getCandidatesUsingPredefinedPoints(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist, List<Integer> predefined_points) {
		
		int n = trajectory.getData().size();
				
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		for(Integer point_position:predefined_points) {
			
			Integer start = point_position - size + 1 ;
			Integer end = point_position;
			
			if(point_position - size + 1 < 0 )
				start = 0;
			
			//Condição caso position seja muito no final da trajetória.
			if(point_position + size > n) 
				end = n - size;
			
			for(; start<=end; start++) {
				
				candidates.addAll(getCandidatesFromTraj(trajectory, train, size, start, mdist));
				
			}
			
		}		
		
		return candidates;
		                                                    
	}
	
	private Integer checkingGoodPointsInNeighborhood(ITrajectory trajectory, Integer actual_position, List<Integer> predefined_points){
		
		if(actual_position + 1 >= trajectory.getData().size())			
			return actual_position;
		
		else if(predefined_points.contains(actual_position + 1))
			return checkingGoodPointsInNeighborhood(trajectory, actual_position+1, predefined_points);
		
		else
			return actual_position+1;
		
	}
	
	@SuppressWarnings("null")
	private List<Range> getrangesUsingNeighbourhood(ITrajectory trajectory, List<ITrajectory> train, double[][][][] mdist, List<Integer> predefined_points){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		int n = trajectory.getData().size();
		
		List<Integer> already_visited_points = new ArrayList<>();
		List<Range> good_ranges = new ArrayList<>();
		
		for(Integer point_position:predefined_points) {
			
			if(!already_visited_points.contains(point_position) || already_visited_points.isEmpty()) {
				
				Integer point_start = point_position - 1;
				Integer point_end = checkingGoodPointsInNeighborhood(trajectory, point_position, predefined_points);

				if(point_position - 1 < 0 )
					point_start = 0;
				
				for(int j=point_start; j<=point_end; j++)
					already_visited_points.add(j);
				
				Range range = new Range(point_start, point_end, point_end-point_start+1);
				good_ranges.add(range);
				
			}			
			
		}	
		
		return good_ranges;
	}
	
	private List<ISubtrajectory> getCandidatesUsingPredefinedRanges(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist, List<Range> ranges){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		int n = trajectory.getData().size();
		
		for(Range range:ranges) {
			
			if(range.getMoveletSize()>=size) {
				
				for (int start = range.getStart(); start <= range.getEnd()-size+1; start++) {

					candidates.addAll(getCandidatesFromTraj(trajectory, train, size, start, mdist));

				} // for (int start = 0; start <= (n - size); start++)
	
			}
			
		}
		
		//SE O RANGE ESTÁ NO LIMITE DO SIZE, ENTÃO CRIA MOVELETS, SENÃO, NÃO
		return candidates;
				
		
	}
	
	private int getBestAlignmentByRanking(double[][] ranksForT, int[] comb) {
		
		double[] rankMerged = new double[ranksForT[0].length];
		for (int i = 0; i < comb.length; i++) {
			for (int j = 0; j < ranksForT[0].length; j++) {
				rankMerged[j] += ranksForT[comb[i]][j];
			}
		}
		/*			
		int minIdx = IntStream.range(0,rankMerged.length)
	            .reduce((i,j) -> rankMerged[i] > rankMerged[j] ? j : i)
	            .getAsInt(); 
	            */
		int minRankIndex = 0;
		for (int j = 1; j < rankMerged.length; j++) {
			if (rankMerged[j] < rankMerged[minRankIndex])
				minRankIndex = j;
		}
		/*
		System.out.println(Arrays.toString(rankMerged));
		System.out.println(minIdx);
		*/
		return minRankIndex;
	}

	public int getArrayIndex(double[] arr,double value) {
	    for(int i=0;i<arr.length;i++)
	        if(arr[i]==value) return i;
	    return -1;
	}
	
	private double[][][][] getBaseCase(ITrajectory trajectory, List<ITrajectory> train){
		int n = trajectory.getData().size();
		int size = 1;
		
		double[][][][] base = new double[(n - size)+1][][][];		
		
		for (int start = 0; start <= (n - size); start++) {
			
			base[start] = new double[train.size()][][];				
			
			for (int i = 0; i < train.size(); i++) {
								
				base[start][i] = new double[dmbt.getDMBP().getNumberOfFeatures()][(train.get(i).getData().size()-size)+1];
						
				for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
					
					double[] distance = dmbt.getDMBP().getDistance(
							trajectory.getData().get(start),
							train.get(i).getData().get(j));
							
					for (int k = 0; k < distance.length; k++) {
					
						base[start][i][k][j] = (distance[k] != Double.MAX_VALUE) ? (distance[k] * distance[k]) : Double.MAX_VALUE;					
					
					} // for (int k = 0; k < distance.length; k++)
					
				} // for (int j = 0; j <= (train.size()-size); j++)
				
			} //for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)

		return base;
	}


	private double[][][][] getNewSize(ITrajectory trajectory, List<ITrajectory> train, double[][][][] base, double[][][][] lastSize, int size) {
		
		int n = trajectory.getData().size();	
		
		for (int start = 0; start <= (n - size); start++) {
						
			for (int i = 0; i < train.size(); i++) {
				
				if (train.get(i).getData().size() >= size) {						
							
					for (int j = 0; j <= (train.get(i).getData().size()-size); j++) {
												
						for (int k = 0; k < lastSize[start][i].length; k++) {
							
							if (lastSize[start][i][k][j] != Double.MAX_VALUE)
								
								lastSize[start][i][k][j] += base[start+size-1][i][k][j+size-1];
						
						} // for (int k = 0; k < distance.length; k++) {
											
					} // for (int j = 0; j <= (train.size()-size); j++)
					
				} // if (train.get(i).getData().size() >= size) 
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int start = 0; start <= (n - size); start++)
		
		return lastSize;
	}
	
	private double[][][][] clone4DArray(double [][][][] source){
		double[][][][] dest = new double[source.length][][][];
		for (int i = 0; i < dest.length; i++) {
			dest[i] = new double[source[i].length][][];
			for (int j = 0; j < dest[i].length; j++) {
				dest[i][j] = new double[source[i][j].length][];
				for (int k = 0; k < dest[i][j].length; k++) {
					dest[i][j][k] = new double[source[i][j][k].length];
					for (int k2 = 0; k2 < source[i][j][k].length; k2++) {
						dest[i][j][k][k2] = source[i][j][k][k2];
					}
				}
			}
		}
		return dest;		
	}
	
	private List<Integer> getPointsInMovelet(List<ISubtrajectory> candidates) {
		
		List<Integer> best_points = new ArrayList<>();

		for(ISubtrajectory candidate:candidates) {
			best_points.add(candidate.getStart());
		}

		return best_points;
	}
	
	private Double checkpointProbability(ITrajectory trajectory, List<ISubtrajectory> candidates, List<Integer> best_points) {

		List<List<Integer>> points_in_movelets = new ArrayList<>();
		for(ISubtrajectory candidate:candidates) {
			
			List<Integer> points_in_movelet = new ArrayList<>();
			for(int i=candidate.getStart(); i < candidate.getStart()+candidate.getSize(); i++)
				points_in_movelet.add(i);
			
			points_in_movelets.add(points_in_movelet);
			
		}
		
		int point_occurence = 0;
		for(List<Integer> list_of_points:points_in_movelets) {
			
			boolean noElementsInCommon = Collections.disjoint(list_of_points, best_points);
			if(!noElementsInCommon)
				point_occurence = point_occurence + 1;
		}
		
		List<Integer> best_points_aux = best_points;

		Boolean point_happen = false;
		Integer point_occurence2 = 0;
		//System.out.print("Traj. Size:" + trajectory.getData().size() + " Points: " + best_points);
		for(Integer point:best_points) {
			
			for(List<Integer> list_of_points:points_in_movelets) {

				if(list_of_points.contains(point)) {

					point_happen = true;
					//System.out.println(" Point: " + point + " List:" + list_of_points);
				}
			}
				
			if(point_happen)
				point_occurence2 = point_occurence2 + 1;
			
		}
		
		//System.out.println("Pontos bons encontrados:" + point_occurence + " movelets ao todo: " + points_in_movelets.size() + " Probabilidade:" + point_occurence/points_in_movelets.size());
		
		//System.out.println("Tamanho da trajetória:" + trajectory.getData().size() + " Número de pontos bons:" + best_points.size() + " Número de movelets boas:" + points_in_movelets.size() + " Probabilidade:" + point_occurence/points_in_movelets.size());
		
		//return (double)point_occurence/points_in_movelets.size();
		return (double)(point_occurence2/best_points.size());
	}
	
	private List<ISubtrajectory> fastMoveletsDiscoveryUsingCacheFirstPivotApproach(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize){
				
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		int n = trajectory.getData().size();
		maxSize = (maxSize == -1) ? n : maxSize;
		
		MyCounter.numberOfCandidates += (maxSize * (maxSize-1) / 2);
		
		/* It starts with the base case
		 * */		
		int size = 1;
				
		double[][][][] base = getBaseCase(trajectory, train);
		if( minSize >= 1 ) {
			candidates.addAll(getCandidatesUsingMDist(trajectory, train, size, base));
			candidates.forEach(x -> AssesQuality(x));
		}			
		
		int size_aux = candidates.size();
		candidates = MoveletsFilterAndRanker.getOneSizeMovelets(candidates);
		size_aux = size_aux - candidates.size();
		List<Integer> best_points = getPointsInMovelet(candidates);
		
		double[][][][] lastSize = clone4DArray(base);			

		/* Tratar o resto dos tamanhos 
		 * */
		for (size = 2; size <= maxSize; size++) {
	
			/* Precompute de distance matrix
			 * */
			double[][][][] newSize = getNewSize(trajectory, train, base, lastSize, size);
			
			/* Create candidates and compute min distances
			 * */
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingPredefinedPoints(trajectory, train, size, newSize, best_points);
			
			if (size >= minSize){
				
				//for (ISubtrajectory candidate : candidatesOfSize) AssesQuality(candidate);				
				candidatesOfSize.forEach(x -> AssesQuality(x));
				
				//candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
				
				candidates.addAll(candidatesOfSize);
			}
		
			lastSize = newSize;
						
		} // for (int size = 2; size <= max; size++)	
	
		base =  null;
		lastSize = null;
		System.out.println("Trajectory:" + trajectory.getTid() + " Number of Candidates:" + (candidates.size()+size_aux));
		
		candidates = MoveletsFilterAndRanker.getShapelets(candidates);
		
		//Double probability = checkpointProbability(trajectory, candidates, best_points); 
		
		//System.out.println(candidates.size() + " " + probability);
		return candidates;
	}
	
	private List<ISubtrajectory> getCandidatesFromTraj(ITrajectory trajectory, List<ITrajectory> train, int size, int start, double[][][][] mdist){
		
		int numberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();
		
		List<ISubtrajectory> list = SubtrajectoryBuilder.build(start, start + size - 1, trajectory, numberOfFeatures, train.size(), exploreDimensions, maxNumberOfFeatures);
			
		double[][][] distancesForAllT = mdist[start];
		
		double distance;
		
		for (int i = 0; i < train.size(); i++) {
				
			double[][] distancesForT = distancesForAllT[i];
			
			double[][] ranksForT = new double[distancesForT.length][];
			
			int limit = train.get(i).getData().size() - size + 1;				
			
			if (limit > 0)
				for (int k = 0; k < numberOfFeatures; k++) {				
					ranksForT[k] = rankingAlgorithm.rank(Arrays.stream(distancesForT[k],0,limit).toArray());
				} // for (int k = 0; k < numberOfFeatures; k++)
			
			int k2 = 0;
			
			int currentFeatures;
					
			if (exploreDimensions){
				currentFeatures = 1;
			} else {
					currentFeatures = numberOfFeatures;
			}
			
			for (;currentFeatures <= maxNumberOfFeatures; currentFeatures++) {
				
				for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) {
											
					int bestPosition = (limit > 0) ? getBestAlignmentByRanking(ranksForT,comb) : -1;
											
					for (int j = 0; j < comb.length; j++) {
						
						distance = (bestPosition >= 0) ? distancesForT[comb[j]][bestPosition] : Double.MAX_VALUE;
						
						list.get(k2).getDistances()[j][i] = 
								(distance != Double.MAX_VALUE) ? Math.sqrt(distance / size) 
															   : Double.MAX_VALUE;
						
					} // for (int j = 0; j < comb.length; j++)
											
					k2++;
					
				} // for (int[] comb : new Combinations(numberOfFeatures,currentFeatures)) 					
				
			} // for (int i = 0; i < train.size(); i++)
			
		} // for (int currentFeatures = 1; currentFeatures <= numberOfFeatures; currentFeatures++)
		
		return list;
		
	}

	private List<ISubtrajectory> getCandidatesUsingPredefinedRanges(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist, Map<Integer,List<Range>> ranges_map){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		int n = trajectory.getData().size();
		
		for (Integer key : ranges_map.keySet()) {
			
			if(key >= size) {
					
				List<Range> ranges = ranges_map.get(key);
				
				for(Range range:ranges) {
					
					for (int start = range.getStart(); start <= range.getEnd()-size+1; start++) {
						
						candidates.addAll(getCandidatesFromTraj(trajectory, train, size, start, mdist));
						
					}
				}
			}
		}

		return candidates;
		
	}
	
	
	private Map<Integer,List<Range>> getRangesUsingNeighbourhood(ITrajectory trajectory, List<ITrajectory> train, double[][][][] mdist, List<Integer> best_points){
		
		List<Integer> already_visited_points = new ArrayList<>();
		Map<Integer,List<Range>> map_of_ranges = new HashMap<Integer,List<Range>>();
		
		for(Integer point_position:best_points) {
			
			if(!already_visited_points.contains(point_position) || already_visited_points.isEmpty()) {
				
				Integer point_start = point_position - 1;
				Integer point_end = checkingGoodPointsInNeighborhood(trajectory, point_position, best_points);

				if(point_position - 1 < 0 )
					point_start = 0;
				
				for(int j=point_start; j<=point_end; j++)
					already_visited_points.add(j);
				
				Range range = new Range(point_start, point_end, point_end-point_start+1);
				
				List<Range> key_range = map_of_ranges.get(point_end-point_start+1);
				
				if(key_range == null)
					key_range = new ArrayList<>();
				
				key_range.add(range);				
				map_of_ranges.put(point_end-point_start+1, key_range);
				
				//This one is necessary to get which is the max size of movelet it is possible to get
				//Using the pre computed ranges
				if(range.getMoveletSize() > limit_size)
					this.limit_size = range.getMoveletSize();
				
			}			
			
		}	
		
		return map_of_ranges;
	}	
	
	private List<ISubtrajectory> fastMoveletsDiscoveryUsingCacheSecondPivotApproach(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){
		
		List<ISubtrajectory> candidates = new ArrayList<>();
		List<ISubtrajectory> candidates_one_size = new ArrayList<>();
		List<ISubtrajectory> pivots_one_size = new ArrayList<>();
		List<ISubtrajectory> candidates_prunned = new ArrayList<>();
		
		int n = trajectory.getData().size();
		maxSize = (maxSize == -1) ? n : maxSize;
		
		MyCounter.numberOfCandidates += (maxSize * (maxSize-1) / 2);
		
		/* It starts with the base case
		 * */		
		int size = 1;
				
		//Calculate the Base Case Table
		double[][][][] base = getBaseCase(trajectory, train);
		
		//Starts with constructing movelets of size one
		candidates_one_size.addAll(getCandidatesUsingMDist(trajectory, train, size, base));
		
		//Gets the quality of each movelet of size one
		candidates_one_size.forEach(x -> AssesQuality(x, random));
		
		//Get only the best movelets of size one
		int max_number_of_features_in_dataset = dmbt.getDMBP().getNumberOfFeatures();
		int number_of_pivots =(int) Math.ceil(n * ((double) porcentage/ (double) 100));
		
		//pivots_one_size = MoveletsFilterAndRanker.getOneSizeMovelets(candidates_one_size, porcentage, maxNumberOfFeatures);
		
		pivots_one_size = MoveletsFilterAndRanker.getOneSizeMovelets(candidates_one_size, number_of_pivots);
		
		//Get the position where the pivots are
		List<Integer> best_points = getPointsInMovelet(pivots_one_size);	
		
		Integer total_size = 0;
		
		//Get all the Neighbourhoods
		Map<Integer,List<Range>> ranges_map = getRangesUsingNeighbourhood(trajectory, train, base, best_points);
		
		//If I include the one size candidates movelets besides the pivots or not
		if(minSize==1 && !only_pivots) {
			
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingPredefinedRanges(trajectory, train, size, base, ranges_map);
			
			total_size = total_size + candidatesOfSize.size();
			
			candidatesOfSize.forEach(x -> AssesQuality(x, random));
			
			candidates.addAll(candidatesOfSize);
			
			total_size = total_size + candidatesOfSize.size();
			
		}else if(minSize==1 && only_pivots)
			candidates.addAll(pivots_one_size);
		
		double[][][][] lastSize = clone4DArray(base);
		/* Tratar o resto dos tamanhos 
		 * */
		for (size = 2; size <= limit_size; size++) {
	
			/* Precompute de distance matrix
			 * */
			double[][][][] newSize = getNewSize(trajectory, train, base, lastSize, size);
			
			/* Create candidates and compute min distances
			 * */
			List<ISubtrajectory> candidatesOfSize = getCandidatesUsingPredefinedRanges(trajectory, train, size, newSize, ranges_map);
			
			total_size = total_size + candidatesOfSize.size();
			
			if (size >= minSize){
				
				//for (ISubtrajectory candidate : candidatesOfSize) AssesQuality(candidate);				
				candidatesOfSize.forEach(x -> AssesQuality(x, random));
				
				//CONFERIR O QUE ACONTECE SE COLOCAR DE VOLTA ESSER RANKER.
				candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
				
				candidates.addAll(candidatesOfSize);
			}
		
			lastSize = newSize;
						
		} // for (int size = 2; size <= max; size++)	
	
		base =  null;
		lastSize = null;

		candidates_prunned = MoveletsFilterAndRanker.getShapelets(candidates);
		
//		System.out.println("Trajectory: " + trajectory.getTid());
//		
//		for (Integer key : ranges_map.keySet()) {
//					
//			List<Range> ranges = ranges_map.get(key);
//			
//			for(Range range:ranges) {
//				
//				System.out.println(range.getStart() + " " + range.getEnd());
//				
//			}
//		}

//		System.out.println(" ");
		System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Pivots: " + pivots_one_size.size() + ". Number of Candidates: " + total_size + ". Limit Size: " + limit_size + ". Total of Movelets: " + candidates_prunned.size() + " Selected Points: " + best_points);
		
		return candidates_prunned;

	}
}
