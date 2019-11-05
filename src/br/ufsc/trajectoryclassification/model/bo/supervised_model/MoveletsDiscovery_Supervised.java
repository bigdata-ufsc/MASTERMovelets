package br.ufsc.trajectoryclassification.model.bo.supervised_model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.concurrent.Callable;

import org.apache.commons.math3.stat.ranking.NaturalRanking;
import org.apache.commons.math3.stat.ranking.RankingAlgorithm;
import org.apache.commons.math3.util.Combinations;
import org.apache.commons.math3.util.Pair;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.bo.movelets.MoveletsFilterAndRanker;
import br.ufsc.trajectoryclassification.model.bo.movelets.SubtrajectoryBuilder;
import br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures.IQualityMeasure;
import br.ufsc.trajectoryclassification.model.bo.pivots.GetOutsidePivots;
import br.ufsc.trajectoryclassification.model.vo.IPoint;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;
import br.ufsc.trajectoryclassification.model.vo.Range;
import br.ufsc.trajectoryclassification.model.vo.Subtrajectory;
import br.ufsc.trajectoryclassification.model.vo.Trajectory;
import br.ufsc.trajectoryclassification.model.vo.features.IFeature;

public class MoveletsDiscovery_Supervised implements Callable<Integer> {
	
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

	private static boolean pivots = false;
	
	private static Boolean attribute_limit = false;
	
	List<Integer> relevant_points_in_trajectory;
	
	private RankingAlgorithm rankingAlgorithm = new NaturalRanking();
			
	public MoveletsDiscovery_Supervised(List<ISubtrajectory> candidates, ITrajectory trajectory, List<ITrajectory> trajectories,
			IDistanceMeasureForSubtrajectory dmbt, IQualityMeasure qualityMeasure, int minSize, int maxSize, boolean cache, boolean exploreDimensions,
			int maxNumberOfFeatures, List<Integer> relevant_points_in_trajectory, boolean pivots, boolean attribute_limit) {
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
		this.relevant_points_in_trajectory = relevant_points_in_trajectory;		
		this.pivots = pivots;
		this.attribute_limit = attribute_limit;

		if (this.maxNumberOfFeatures < 1 || this.maxNumberOfFeatures > dmbt.getDMBP().getNumberOfFeatures())
			this.maxNumberOfFeatures = dmbt.getDMBP().getNumberOfFeatures();

	}
	
	@Override
	public Integer call() throws Exception {
		
		measureShapeletCollection();
		
		return 0;
	}
	
	public void measureShapeletCollection() {

		
		List<ISubtrajectory> candidates = new ArrayList<>();
		
		this.random = new Random(trajectory.getTid());
		
		if(!this.relevant_points_in_trajectory.isEmpty()) {
			if(pivots && maxSize!=-3)
				candidates = SupervisedMoveletsDiscovery_with_Pivots(trajectory, trajectories, minSize, maxSize, random);
			else if(maxSize==-3)
				candidates = SupervisedMoveletsDiscovery_With_Log(trajectory, trajectories, minSize, maxSize, random);
			else
				candidates = SupervisedMoveletsDiscovery(trajectory, trajectories, minSize, maxSize, random);
		}

		for (ISubtrajectory candidate : candidates) {
			/*
			 * STEP 1: COMPUTE DISTANCES, IF NOT COMPUTED YET
			 */
			//if (candidate.getDistances() == null)		
			//	ComputeDistances(candidate);
			
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
	
	private Integer checkingGoodPointsInNeighborhood(ITrajectory trajectory, Integer actual_position, List<Integer> predefined_points){
		
		if(actual_position + 1 >= trajectory.getData().size())			
			return actual_position;
		
		else if(predefined_points.contains(actual_position + 1))
			return checkingGoodPointsInNeighborhood(trajectory, actual_position+1, predefined_points);
		
		else
			return actual_position+1;
		
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
	
	private List<ISubtrajectory> getCandidatesUsingMDist(ITrajectory trajectory, List<ITrajectory> train, int size, double[][][][] mdist) {
		
		int n = trajectory.getData().size();
				
		List<ISubtrajectory> candidates = new ArrayList<>();

		for (int start = 0; start <= (n - size); start++) {
			
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
			
			candidates.addAll(list);

		} // for (int start = 0; start <= (n - size); start++)
		
		return candidates;
		
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
	
	public Map<String,List<Integer>> getPOISInTrajectory(){

		Map<String,List<Integer>> pois_in_traj = new HashMap<String,List<Integer>>();
		
		for(int j=0; j<trajectory.getData().size(); j++) {
			
			String poi = trajectory.getData().get(j).getFeature("poi").toString();
			
			if(pois_in_traj.containsKey(poi)) {
				
				List<Integer> positions = pois_in_traj.get(poi);
				positions.add(j);
								
				pois_in_traj.put(poi, positions);
				
			}else {
				List<Integer> positions =  new ArrayList<>();
				positions.add(j);
				pois_in_traj.put(poi, positions);
			}
			
		}
		
		return pois_in_traj;
		
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
	
	private Integer checkEndOfTrajectorySegment(Integer actual_position, List<Integer> points){
		
		if(points.contains(actual_position + 1))
			return checkEndOfTrajectorySegment(actual_position+1, points);
		
		else
			return actual_position;
		
	}
	
	private Map<Integer,List<Range>> getTrajectoryAreaSupervised(List<Integer> points){
		
		//sort the list of integer
		Collections.sort(points);
		
		List<Integer> already_visited_points = new ArrayList<>();
		Map<Integer,List<Range>> map_of_ranges = new HashMap<Integer,List<Range>>();
		
		for(Integer point_position:points) {
			
			if(!already_visited_points.contains(point_position) || already_visited_points.isEmpty()) {
				
				Integer point_start = point_position;
				Integer point_end = checkEndOfTrajectorySegment(point_position, points);
				
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
	
	public List<ITrajectory> getSubtrajectoriesFromRanges(Map<Integer,List<Range>> ranges_map){
		
		List<ITrajectory> neighborhood_subtrajectories = new ArrayList<>();
		
		int id_ = 1;
		for (Integer key : ranges_map.keySet()) {
			
			for(Range range:ranges_map.get(key)) {
				
				int start = range.getStart();
				List<IPoint> range_points = new ArrayList<>();
				
				for(int i =0; i<key;i++)range_points.add(trajectory.getData().get(start+i));									
				
				ITrajectory a_subtrajectory = new Trajectory(id_, range_points, trajectory.getLabel());
				neighborhood_subtrajectories.add(a_subtrajectory);
				id_++;
				
			}
			
		}
		
		return neighborhood_subtrajectories;
		
	}
	
	public List<ISubtrajectory> SupervisedMoveletsDiscovery(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){
		
		//GET THE RANGES IN WICH WE ARE SUPPOSES TO EXTRACT CANDIDATES
		Map<Integer,List<Range>> ranges_map = getTrajectoryAreaSupervised(relevant_points_in_trajectory);

		//FOR EACH NEIGHBORHOOD, FIND THE CORRESPONDENT SUBTRAJECTORY
		List<ITrajectory> neighborhood_subtrajectories = getSubtrajectoriesFromRanges(ranges_map);
		
		List<double[][][][]> base_cases = new ArrayList<>();
		
		//CONSTRUCT THE BASE CASES
		for(int i =0; i<neighborhood_subtrajectories.size(); i++) base_cases.add(getBaseCase(neighborhood_subtrajectories.get(i), train));
		
		int total_size = 0;

		List<ISubtrajectory> final_candidates = new ArrayList<>();
		//FOR EACH BASE CASE, EXTRACT THE MOVELETS		
		
		for(int i = 0; i< base_cases.size(); i++) {

			int size = 1;
			ITrajectory trajectory_ = neighborhood_subtrajectories.get(i);
			
			List<ISubtrajectory> candidates = new ArrayList<>();
			
			if( minSize <= 1 ) {
				candidates.addAll(getCandidatesUsingMDist(trajectory_, train, size, base_cases.get(i)));
				candidates.forEach(x -> AssesQuality(x, random));
			}				
			
			double[][][][] lastSize = clone4DArray(base_cases.get(i));			

			total_size = total_size + candidates.size();
			
			int maxSize_ = trajectory_.getData().size();
			/* Tratar o resto dos tamanhos 
			 * */
			
			for (size = 2; size <= maxSize_; size++) {

				/* Precompute de distance matrix
				 * */
				double[][][][] newSize = getNewSize(trajectory_, train, base_cases.get(i), lastSize, size);
				
				/* Create candidates and compute min distances
				 * */			
				List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory_, train, size, newSize);
				
				total_size = total_size + candidatesOfSize.size();
				
				if (size >= minSize){
					
					//for (ISubtrajectory candidate : candidatesOfSize) AssesQuality(candidate);				
					candidatesOfSize.forEach(x -> AssesQuality(x, random));
					
					//candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
					
					candidates.addAll(candidatesOfSize);
				}
				
				lastSize = newSize;
							
			} // for (int size = 2; size <= max; size++)	
			
			lastSize = null;

			candidates = MoveletsFilterAndRanker.getShapelets(candidates);
			final_candidates.addAll(candidates);
		}


		System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Ranges: " + neighborhood_subtrajectories.size() + ". Number of Candidates: " + total_size + ". Limit Size: " + limit_size + ". Total of Movelets: " + final_candidates.size() + ". Max number of Features: " + maxNumberOfFeatures);
		
		base_cases = null;
		return final_candidates;
		
	}

	public List<ISubtrajectory> SupervisedMoveletsDiscovery_with_Pivots(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){

		
		//GET THE RANGES IN WICH WE ARE SUPPOSES TO EXTRACT CANDIDATES
		Map<Integer,List<Range>> supervised_ranges = getTrajectoryAreaSupervised(relevant_points_in_trajectory);

		//FOR EACH NEIGHBORHOOD, FIND THE CORRESPONDENT SUBTRAJECTORY
		List<ITrajectory> neighborhood_subtrajectories = getSubtrajectoriesFromRanges(supervised_ranges);
		
		List<Pivot_Supervised> supervised_pivots = new ArrayList<>();

		List<ISubtrajectory> all_candidates_size_one = new ArrayList<>();
		
		for(int j=0; j<neighborhood_subtrajectories.size(); j++) {
			
			List<ISubtrajectory> candidates_size_one = new ArrayList<>();
			
			//Identify the id of the pivot range
			Pivot_Supervised a_pivot = new Pivot_Supervised(neighborhood_subtrajectories.get(j).getTid());		
			
			//Set the trajectory to the pivot range
			a_pivot.setTrajectory(neighborhood_subtrajectories.get(j));
			
			//Set the base case for the pivot range
			a_pivot.setBaseCase(getBaseCase(neighborhood_subtrajectories.get(j), train));
			
			//For last, identify the candidates of size one for each pivot range
			candidates_size_one = getCandidatesUsingMDist(neighborhood_subtrajectories.get(j), train, 1, getBaseCase(neighborhood_subtrajectories.get(j), train));
			candidates_size_one.forEach(x -> AssesQuality(x, random));
			
			a_pivot.setCandidates_Size_One(candidates_size_one);
			
			candidates_size_one.forEach(x -> all_candidates_size_one.add(x));
			
			supervised_pivots.add(a_pivot);			
			
		}

		//GET THE TOTAL SIZE OF THE TRAJECTORY
		int n = 0;
		
		for(ITrajectory traj:neighborhood_subtrajectories)
			n = n+traj.getData().size();
		
		int number_of_pivots =(int) Math.ceil(n * ((double) 10/ (double) 100));

		List<ISubtrajectory> pivots_one_size = MoveletsFilterAndRanker.getOneSizeMovelets(all_candidates_size_one, number_of_pivots);

		List<ISubtrajectory> final_candidates = new ArrayList<>();

		for(ISubtrajectory pivot:pivots_one_size) {
			
			for( int p = 0; p<supervised_pivots.size(); p++) {
				
				if(supervised_pivots.get(p).getID()==pivot.getTrajectory().getTid())
					supervised_pivots.get(p).best_points.add(pivot.getStart());
				
			}
			
		}
		
		List<ISubtrajectory> total_candidates = new ArrayList<>();

		Integer total_size = 0;
		int local_limit_size = 0;
		int higher_local_limit_size = 0;
		
		for(Pivot_Supervised pivot_set:supervised_pivots) {
			
			local_limit_size = 0;
			ITrajectory trajectory_ = pivot_set.getTrajectory();
			List<ISubtrajectory> candidates_prunned = new ArrayList<>();
			
			double[][][][] base = pivot_set.getBaseCase();
			
			//Get all the Neighbourhoods for each trajetory part
			Map<Integer,List<Range>> ranges_map = getRangesUsingNeighbourhood(trajectory_, train, base, pivot_set.getBestPoints());
			
			for(Integer key : ranges_map.keySet()) {
				if(local_limit_size < key)
					local_limit_size = key;
			}

			if(local_limit_size>higher_local_limit_size)
				higher_local_limit_size = local_limit_size;
			
			int size = 1;
			
			//Start generating the candidates
			List<ISubtrajectory> candidatesOfSizeOne = getCandidatesUsingPredefinedRanges(trajectory_, train, size, base, ranges_map);
				
				
			candidatesOfSizeOne.forEach(x -> AssesQuality(x, random));
			candidates_prunned = MoveletsFilterAndRanker.getShapelets(candidatesOfSizeOne);
			total_candidates.addAll(candidates_prunned);
				
			total_size = total_size + candidatesOfSizeOne.size();
				
			double[][][][] lastSize = clone4DArray(base);
			
			/* Tratar o resto dos tamanhos 
			 * */
			for (size = 2; size <= local_limit_size; size++) {
		
				/* Precompute de distance matrix
				 * */
				double[][][][] newSize = getNewSize(trajectory_, train, base, lastSize, size);
				
				/* Create candidates and compute min distances
				 * */
				List<ISubtrajectory> candidatesOfSize = getCandidatesUsingPredefinedRanges(trajectory_, train, size, newSize, ranges_map);
				
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
			total_candidates.addAll(candidates_prunned);
			
		}
		System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Ranges: " + neighborhood_subtrajectories.size() + ". Number of Candidates: " + total_size + ". Higher limit Size: " + higher_local_limit_size + ". Total of Movelets: " + total_candidates.size() + ". Max number of Features: " + maxNumberOfFeatures);
		
		return total_candidates;
		
	}
	
	//Dumb replication from the previous function, need to refactor it later.	
	public List<ISubtrajectory> SupervisedMoveletsDiscovery_With_Log(ITrajectory trajectory, List<ITrajectory> train, int minSize, int maxSize, Random random){
		
		//GET THE RANGES IN WICH WE ARE SUPPOSES TO EXTRACT CANDIDATES
		Map<Integer,List<Range>> ranges_map = getTrajectoryAreaSupervised(relevant_points_in_trajectory);

		//FOR EACH NEIGHBORHOOD, FIND THE CORRESPONDENT SUBTRAJECTORY
		List<ITrajectory> neighborhood_subtrajectories = getSubtrajectoriesFromRanges(ranges_map);
		
		List<double[][][][]> base_cases = new ArrayList<>();
		
		//CONSTRUCT THE BASE CASES
		for(int i =0; i<neighborhood_subtrajectories.size(); i++) base_cases.add(getBaseCase(neighborhood_subtrajectories.get(i), train));
		
		int total_size = 0;

		List<ISubtrajectory> final_candidates = new ArrayList<>();
		//FOR EACH BASE CASE, EXTRACT THE MOVELETS		
		//DO NOT SURPASS THE SIZE LIMIT OF LOG
		int global_maxSize = (int) Math.ceil(Math.log(trajectory.getData().size()))+1;

		int higher_local_size = 0;
		
		for(int i = 0; i< base_cases.size(); i++) {

			int size = 1;
			ITrajectory trajectory_ = neighborhood_subtrajectories.get(i);
			
			List<ISubtrajectory> candidates = new ArrayList<>();
			
			if( minSize <= 1 ) {
				candidates.addAll(getCandidatesUsingMDist(trajectory_, train, size, base_cases.get(i)));
				candidates.forEach(x -> AssesQuality(x, random));
			}				
			
			double[][][][] lastSize = clone4DArray(base_cases.get(i));			

			total_size = total_size + candidates.size();
			
			//But first, I need to grant that the max_value will not surpass the size of the trajectory segment
			int local_maxSize_ = trajectory_.getData().size();
			if(local_maxSize_ > higher_local_size) higher_local_size = local_maxSize_;
			
			/* Tratar o resto dos tamanhos 
			 * */
			for (size = 2; size <= local_maxSize_ && size <= total_size; size++) {

				/* Precompute de distance matrix
				 * */
				double[][][][] newSize = getNewSize(trajectory_, train, base_cases.get(i), lastSize, size);
				
				/* Create candidates and compute min distances
				 * */			
				List<ISubtrajectory> candidatesOfSize = getCandidatesUsingMDist(trajectory_, train, size, newSize);
				
				total_size = total_size + candidatesOfSize.size();
				
				if (size >= minSize){
					
					//for (ISubtrajectory candidate : candidatesOfSize) AssesQuality(candidate);				
					candidatesOfSize.forEach(x -> AssesQuality(x, random));
					
					//candidatesOfSize = MoveletsFilterAndRanker.getShapelets(candidatesOfSize);
					
					candidates.addAll(candidatesOfSize);
				}
				
				lastSize = newSize;
							
			} // for (int size = 2; size <= max; size++)	
			
			lastSize = null;

			candidates = MoveletsFilterAndRanker.getShapelets(candidates);
			final_candidates.addAll(candidates);
		}


		System.out.println("Trajectory: " + trajectory.getTid() + ". Trajectory Size: " + trajectory.getData().size() + ". Number of Ranges: " + neighborhood_subtrajectories.size() + ". Number of Candidates: " + total_size + ". Limit Size: " + global_maxSize + ". Total of Movelets: " + final_candidates.size()+ ". Max number of Features: " + maxNumberOfFeatures);
		
		base_cases = null;
		return final_candidates;
		
	}
	
}

