package ca.mcgill.ecse211.project.game;

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
  
  Initialization{
      @Override
      public GameState nextState() {
        return LightLocalization;
      }
      
    },
  
  TUNNEL{
      @Override
      public GameState nextState() {
        return LightLocalization;
      }
      
    },
  ColorLocalization{
      @Override
      public GameState nextState() {
        return LightLocalization;
      }
      
    },
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
