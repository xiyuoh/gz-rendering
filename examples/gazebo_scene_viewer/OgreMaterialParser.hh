/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#ifndef _IGNITION_OGRE_MATERIAL_PARSER_HH_
#define _IGNITION_OGRE_MATERIAL_PARSER_HH_

#include <string>

namespace ignition
{
  namespace rendering
  {
    class ConfigNode;
    class ConfigLoader;

    class OgreMaterialParser
    {
      public: OgreMaterialParser();

      public: virtual ~OgreMaterialParser();

      public: void Load(const std::string &_path);

      public: std::string MaterialAsJson() const;

      private: ConfigLoader *configLoader;
    };

    class ConfigNode
    {
      public: ConfigNode(ConfigNode *_parent, const std::string &_name = "");

      public: virtual ~ConfigNode();

      public: void SetName(const std::string &_name);

      public: std::string Name() const;

      public: void AddValue(const std::string &_value);

      public: void ClearValues();

      public: std::vector<std::string> &Values();

      public: const std::string &Value(unsigned int _index = 0);

      public: ConfigNode *AddChild(const std::string &_name,
          bool _replaceExisting = false);

      public: ConfigNode *FindChild(const std::string &_name,
           bool _recursive = false);

      public:  std::vector<ConfigNode *> &Children();

      public:  ConfigNode *Child(unsigned int _index = 0);

      public: void SetParent(ConfigNode *_newParent);

      public:  ConfigNode *Parent();

      private: std::string name;

      private: std::vector<std::string> values;

      private: std::vector<ConfigNode *> children;

      private: ConfigNode *parent;

      // \brief The last child node's index found with a call to FindChild()
      private: int lastChildFound;

      private: std::vector<ConfigNode *>::iterator selfNode;

      private: bool removeSelf;
    };

    class ConfigLoader
    {
      public: static void LoadAllFiles(ConfigLoader *_c,
          const std::string &_path);

      public: ConfigLoader(const std::string &_fileEnding);

      public: virtual ~ConfigLoader();

      // For a line like
      // entity animals/dog
      // {
      //    ...
      // }
      // The type is "entity" and the name is "animals/dog"
      // Or if animal/dog was not there then name is ""
      public: virtual ConfigNode *ConfigScript (const std::string &_name);

      public: virtual std::map <std::string, ConfigNode *> AllConfigScripts();

      public: virtual void ParseScript(std::ifstream &_stream);

      public: std::string fileEnding;

      protected: float LoadOrder;
      // like "*.object"

      protected: std::map <std::string, ConfigNode *> scriptList;

      protected: enum Token
      {
        TOKEN_TEXT,
        TOKEN_NEWLINE,
        TOKEN_OPENBRACE,
        TOKEN_CLOSEBRACE,
        TOKEN_EOF,
      };

      protected: Token tok, lastTok;
      protected: std::string tokVal, lastTokVal;

      protected: void ParseNodes(std::ifstream &_stream, ConfigNode *_parent);
      protected: void NextToken(std::ifstream &_stream);
      protected: void SkipNewLines(std::ifstream &_stream);

      protected: virtual void ClearScriptList();
    };
  }
}

#endif
