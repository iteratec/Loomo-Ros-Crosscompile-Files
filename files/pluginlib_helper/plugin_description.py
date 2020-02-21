class PluginDescription:
  """
  Class describing a plugin.
  Replicates the information present in the plugin description file XML. 
  """
  lookup_name = ""
  class_type = ""
  base_class = ""
  package = ""
  description = ""
  library_name = ""
  manifest_path = ""
  def __str__(self):
    s = "{ package=%s; manifest_path=%s; library_name=%s; \
lookup_name=%s; class_type=%s; base_class=%s; description='%s' }"\
    %(self.package, self.manifest_path, self.library_name, \
    self.lookup_name, self.class_type, self.base_class, self.description)
    return s
