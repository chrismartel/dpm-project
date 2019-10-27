package ca.mcgill.ecse211.project;

public enum GameState {

  UltrasonicLocalization{

    @Override
    public GameState nextState() {
      return LightLocalization;
    }},
  LightLocalization{

    @Override
    public GameState nextState() {
      return Navigation;
    }},
  Navigation{

    @Override
    public GameState nextState() {
      return null;
    }},
  Avoidance{

    @Override
    public GameState nextState() {
      return Navigation;
    }},
  Launch{

    @Override
    public GameState nextState() {
      return Navigation;
    }};
  
  public abstract GameState nextState();
}
