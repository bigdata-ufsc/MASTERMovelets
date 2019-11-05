package br.ufsc.trajectoryclassification.model.vo;


public interface IRange {
	
	public int getStart();
	
	public int getEnd();
	
	public int getMoveletSize();
	
	public void setStart(int start);
	
	public void setEnd(int end);
	
	public void setMoveletSize(int moveletSize);
	
	String toString();
	
}
