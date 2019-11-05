package br.ufsc.trajectoryclassification.model.bo.supervised_model;

import java.util.ArrayList;
import java.util.List;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;
import br.ufsc.trajectoryclassification.model.vo.ITrajectory;

public class Pivot_Supervised {

	private int id;
	
	private ITrajectory a_subtrajectory;
	
	private List<ISubtrajectory> candidates_size_one = new ArrayList<>();

	private double[][][][] base_case;
	
	List<Integer> best_points = new ArrayList<>();
	
	public Pivot_Supervised(int id) {
		this.id = id;
	}
	
	public void setTrajectory(ITrajectory trajectory) {
		this.a_subtrajectory = trajectory;
	}
	
	public void setBaseCase(double[][][][] base_case) {
		this.base_case = base_case;
	}
	
	public void setCandidates_Size_One(List<ISubtrajectory> candidates_size_one) {
		this.candidates_size_one = candidates_size_one;
	}
	
	public List<Integer> getBestPoints() {
		return best_points;
	}
	
	public List<ISubtrajectory> getCandidates_Size_One() {
		return candidates_size_one;
	}
	
	public double[][][][] getBaseCase() {
		return base_case;
	}
	
	public ITrajectory getTrajectory() {
		return a_subtrajectory;
	}
	
	public int getID() {
		return id;
	}

}
