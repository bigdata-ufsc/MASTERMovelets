package br.ufsc.trajectoryclassification.model.bo.movelets.QualityMeasures;

import java.util.Random;

import br.ufsc.trajectoryclassification.model.vo.ISubtrajectory;

public interface IQualityMeasure {
	
	public void assesQuality(ISubtrajectory candidate);
	
	public void assesQuality(ISubtrajectory candidate, Random random);

}
