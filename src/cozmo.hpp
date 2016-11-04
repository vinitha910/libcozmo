namespace cozmo {
  void setGeometry(const BodyNodePtr& bn);
  BodyNode* makeRootBody(const SkeletonPtr& cozmo, const std::string& name);
  SkeletonPtr* createCubeCozmo();
}
