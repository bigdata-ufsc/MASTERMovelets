package br.ufsc.trajectoryclassification.model.bo.movelets;

import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.commons.collections4.SetUtils;
import org.apache.commons.math3.distribution.IntegerDistribution;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.stat.correlation.PearsonsCorrelation;
import org.apache.commons.math3.stat.correlation.SpearmansCorrelation;

import br.ufsc.trajectoryclassification.model.bo.dmbs.IDistanceMeasureForSubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.utils.ProgressBar;
import br.ufsc.trajectoryclassification.utils.Utils;

public class MoveletsFilterAndRanker {

	public static boolean areSelfSimilar(ISubtrajectory candidate, ISubtrajectory subtrajectory,
			double selfSimilarityProp) {
		
		//return false;
		
		// If their tids are different return false
		
		if (candidate.getTrajectory().getTid() != subtrajectory.getTrajectory().getTid())
			return false;

		else if (candidate.getStart() < subtrajectory.getStart()) {

			if (candidate.getEnd() < subtrajectory.getStart())
				return false;

			if (selfSimilarityProp == 0)
				return true;

			double intersection = (candidate.getEnd() - subtrajectory.getStart())
					/ (double) Math.min(candidate.getSize(), subtrajectory.getSize());

			return intersection >= selfSimilarityProp;

		} else {

			if (subtrajectory.getEnd() < candidate.getStart())
				return false;

			if (selfSimilarityProp == 0)
				return true;

			double intersection = (subtrajectory.getEnd() - candidate.getStart())
					/ (double) Math.min(candidate.getSize(), subtrajectory.getSize());

			return intersection >= selfSimilarityProp;

		}

	}

	public static boolean searchIfSelfSimilarity(ISubtrajectory candidate, List<ISubtrajectory> list,
			double selfSimilarityProp) {

		for (ISubtrajectory s : list) {
			if (areSelfSimilar(candidate, s, selfSimilarityProp))
				return true;
		}

		return false;
	}

	private static boolean isCovered(double[] point, double[] limits){
		
		int dimensions = limits.length;
		
		for (int i = 0; i < dimensions; i++) {
			if (point[i] >= limits[i])
				return false;
		}
		
		return true;
	}

	public static Set<Integer> getIndexesLowerSplitPoint(double[][] distances, double[] splitpoints ){
		Set<Integer> indexes = new HashSet();
		
		RealMatrix rm = new Array2DRowRealMatrix(distances);
		
		for (int i = 0; i < distances[0].length; i++) {
			if ( Utils.isCovered(rm.getColumn(i),splitpoints) )			
				indexes.add(i);
			}
		
		return indexes;		
	}
	
	public static List<ISubtrajectory> noveltyFilter(List<ISubtrajectory> movelets) {


		List<ISubtrajectory> noveltyShapelets = new ArrayList<>();
		Set<Integer> allCovered = new HashSet<Integer>();
		
		//ProgressBar progressBar = new ProgressBar("Movelet Pruning");
		//int total = movelets.get(0).getDistances()[0].length;
		//progressBar.update( allCovered.size(), total );
		
		for (int i = 0; i < movelets.size(); i++) {
			double[][] distances = movelets.get(i).getDistances();
			double[] splitpoint = movelets.get(i).getSplitpoints();
			Set<Integer> currentCovered = getIndexesLowerSplitPoint(distances, splitpoint);
			
			if ( ! SetUtils.difference(currentCovered, allCovered).isEmpty()){
				noveltyShapelets.add(movelets.get(i));
				allCovered.addAll(currentCovered);
				//progressBar.update( allCovered.size(), total );
			}
		
		}
		
		return noveltyShapelets;
	}

	public static List<ISubtrajectory> Redundance2(List<ISubtrajectory> movelets, IDistanceMeasureForSubtrajectory dmbs) {

		double[] distances;
		
		for (int i = 0; i < movelets.size(); i++){
			
			ISubtrajectory m1 = movelets.get(i); 
			
			for (int j = i+1; j < movelets.size(); j++){
			
				ISubtrajectory m2 = movelets.get(j);
				
				if ( m1.getSize() == m2.getSize() && Arrays.equals(m1.getPointFeatures(), m2.getPointFeatures())) {
					distances = dmbs.getDistance(m1, m2);
					
					if ( isCovered( distances, m1.getSplitpoints() ) ){
						
						movelets.remove(j);
						j--;
						
					} // if ( isCovered( distances, m1.getSplitpoints() ) )					
					
				} // if ( m1.getSize() == m2.getSize() && Arrays.equals(m1.getPointFeatures(), m2.getPointFeatures()))				
				
			} // for (int j = i+1; j < movelets.size(); j++)			

		} // for (int i = 0; i < movelets.size(); i++)
		
		return movelets;
	}

	public static List<ISubtrajectory> noveltyFilter1(List<ISubtrajectory> movelets, int moveletsPerTrajectory) {

		List<ISubtrajectory> pruned = new ArrayList<>();
		
		if (moveletsPerTrajectory <= 0)
			return movelets;
		
		else {

			List<ISubtrajectory> candidates = new ArrayList<>(movelets);
			
			for (int i = 0; i < moveletsPerTrajectory; i++) {
				pruned.addAll(noveltyFilterInterno(candidates));
				candidates.removeAll(pruned);
			}
			
		}
		
		return pruned;
	}
	
	public static List<ISubtrajectory> noveltyFilterInterno(List<ISubtrajectory> shapelets) {


		List<ISubtrajectory> noveltyShapelets = new ArrayList<>();
		Set<Integer> allCovered = new HashSet<Integer>();
		Set<Integer> difference = new HashSet<Integer>();
		
		Set<Integer> maxDifference = new HashSet<Integer>();
				
		while (true) {
			
			ISubtrajectory maxDifferenceShapelet = null;
			maxDifference.clear();
			
			for (int i = 0; i < shapelets.size(); i++) {
				double[][] distances = shapelets.get(i).getDistances();
				double[] splitpoint = shapelets.get(i).getSplitpoints();
				Set<Integer> currentCovered = getIndexesLowerSplitPoint(distances, splitpoint);
				
				difference = SetUtils.difference(currentCovered, allCovered);
							
				if (difference.size() > maxDifference.size()){
					maxDifference = difference;
					maxDifferenceShapelet = shapelets.get(i);
				}
			}
			
			
		
		if (maxDifference.size() > 0){
			noveltyShapelets.add(maxDifferenceShapelet);
			allCovered.addAll(maxDifference);			
			}
		else 
			break;
		
		}
		
		return noveltyShapelets;
	}
	
	

	public static List<ISubtrajectory> getShapelets(List<ISubtrajectory> candidates) {

		List<ISubtrajectory> orderedCandidates = rankCandidates(candidates);

		return getBestShapelets(orderedCandidates);
	}

	public static List<ISubtrajectory> getCandidatesWithAllFeatures(List<ISubtrajectory> candidates, int maxNumberOfFeatures) {
		
		List<ISubtrajectory> candidates_with_all_features = new ArrayList<>();
		
		for(ISubtrajectory candidate : candidates) {
			
			if(candidate.getPointFeatures().length == maxNumberOfFeatures)
				candidates_with_all_features.add(candidate);
			
		}
		
		return candidates_with_all_features;
	}
	
	public static List<ISubtrajectory> getOneSizeMovelets(List<ISubtrajectory> candidates, int pivot_number) {

		List<ISubtrajectory> orderedCandidates = rankCandidates(candidates);
		
		List<ISubtrajectory> bestCandidates = getBestShapelets(orderedCandidates);

		List<ISubtrajectory> bestCandidates_Pivots = bestCandidates.subList(0, pivot_number);
		
		return bestCandidates_Pivots;
	}
	
	public static List<ISubtrajectory> getOneSizeMovelets(List<ISubtrajectory> candidates) {

		List<ISubtrajectory> orderedCandidates = rankCandidates(candidates);
		
		List<ISubtrajectory> bestCandidates = getBestShapelets(orderedCandidates);

		List<ISubtrajectory> bestCandidatesHalf = bestCandidates.subList(0, bestCandidates.size()/10);

		return bestCandidatesHalf;
	}
	
	public static List<ISubtrajectory> getOneSizeMovelets(List<ISubtrajectory> candidates, int p, int maxNumberOfFeatures) {

		//It needs to select only the movelet candidates of size one that cover all the possible dimensions
		List<ISubtrajectory> candidates_with_all_features = getCandidatesWithAllFeatures(candidates, maxNumberOfFeatures);
		
		Collections.shuffle(candidates_with_all_features, new Random(1)); 
		
		List<ISubtrajectory> orderedCandidates = rankCandidates(candidates_with_all_features);		

		List<ISubtrajectory> bestCandidates = orderedCandidates.subList(0, (int) Math.ceil(orderedCandidates.size() * ((double) p/ (double) 100)));
		
		if(bestCandidates.size()==0) {
			
			bestCandidates = orderedCandidates.subList(0, 1);
			
		}
		
		return bestCandidates;		
		
	}

	
	public static List<ISubtrajectory> rankCandidates(List<ISubtrajectory> candidates) {

		List<ISubtrajectory> orderedCandidates = new ArrayList<>(candidates);
		
		orderedCandidates.removeIf(e -> e == null);
		
		orderedCandidates.sort(new Comparator<ISubtrajectory>() {
			@Override
			public int compare(ISubtrajectory o1, ISubtrajectory o2) {
				
				return o1.getQuality().compareTo(o2.getQuality());				
				
			}
		});

		return orderedCandidates;
	}

	private static List<ISubtrajectory> getBestShapelets(List<ISubtrajectory> shapeletsRanked) {

		return getBestShapelets(shapeletsRanked, 0);

	}

	public static List<ISubtrajectory> getBestShapelets(List<ISubtrajectory> rankedCandidates,
			double selfSimilarityProp) {

		// Realiza o loop até que acabem os atributos ou até que atinga o número
		// máximo de nBestShapelets
		// Isso é importante porque vários candidatos bem rankeados podem ser
		// selfsimilares com outros
		// que tiveram melhor score;
		for (int i = 0; (i < rankedCandidates.size()); i++) {

			// Se a shapelet candidata tem score 0 então já termina o processo
			// de busca
			if (rankedCandidates.get(i).getQuality().hasZeroQuality())
				return rankedCandidates.subList(0, i);

			ISubtrajectory candidate = rankedCandidates.get(i);

			// Removing self similar
			if (searchIfSelfSimilarity(candidate, rankedCandidates.subList(0, i), selfSimilarityProp)) {
				rankedCandidates.remove(i);
				i--;
			}

		}

		return rankedCandidates;
	}

}
