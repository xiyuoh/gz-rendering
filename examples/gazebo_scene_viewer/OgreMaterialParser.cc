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

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <exception>
#include <fstream>

#include <boost/filesystem.hpp>

#include "OgreMaterialParser.hh"

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
OgreMaterialParser::OgreMaterialParser()
{
  this->configLoader = new ConfigLoader(".material");
}

/////////////////////////////////////////////////
OgreMaterialParser::~OgreMaterialParser()
{
  delete this->configLoader;
  this->configLoader = NULL;
}

/////////////////////////////////////////////////
void OgreMaterialParser::Load(const std::string &_path)
{
  ConfigLoader::LoadAllFiles(this->configLoader, _path);
}

/////////////////////////////////////////////////
std::string OgreMaterialParser::MaterialAsJson() const
{
  std::string jsonStr = "{";

  std::map<std::string, ConfigNode *> scripts =
      this->configLoader->AllConfigScripts();

  bool first = true;
  for (auto &it : scripts)
  {
    std::string name = it.first;
    ConfigNode *node = it.second;

    ConfigNode *techniqueNode = node->FindChild("technique");
    if (techniqueNode)
    {
      ConfigNode *passNode = techniqueNode->FindChild("pass");
      if (passNode)
      {
        if (!first)
          jsonStr += ", ";
        else
          first = false;

        std::size_t index = name.rfind(" ");
        if (index != std::string::npos)
        {
          name = name.substr(index+1);
        }
        jsonStr += "\"" + name + "\":{";

        ConfigNode *ambientNode = passNode->FindChild("ambient");
        if (ambientNode)
        {
          std::stringstream ss;
          std::vector<std::string> values = ambientNode->Values();
          if (values.size() == 1)
            ss << "\"";
          for (unsigned int i = 0; i < values.size(); ++i)
          {
            std::string value = ambientNode->Value(i);
            if (value[0] == '.')
              value = '0' + value;
            ss << value;
            if (i != values.size() - 1)
              ss << ",";
          }
          if (values.size() == 1)
            ss << "\"";
          jsonStr += "\"ambient\":[" + ss.str() + "],";
        }

        ConfigNode *diffuseNode = passNode->FindChild("diffuse");
        if (diffuseNode)
        {
          std::stringstream ss;
          std::vector<std::string> values = diffuseNode->Values();
          if (values.size() == 1)
            ss << "\"";
          for (unsigned int i = 0; i < values.size(); ++i)
          {
            std::string value = diffuseNode->Value(i);
            if (value[0] == '.')
              value = '0' + value;
            ss << value;
            if (i != values.size() - 1)
              ss << ",";
          }
          if (values.size() == 1)
            ss << "\"";
          jsonStr += "\"diffuse\":[" + ss.str() + "],";
        }

        ConfigNode *specularNode = passNode->FindChild("specular");
        if (specularNode)
        {
          std::stringstream ss;
          std::vector<std::string> values = specularNode->Values();
          if (values.size() == 1)
            ss << "\"";
          for (unsigned int i = 0; i < values.size(); ++i)
          {
            std::string value = specularNode->Value(i);
            if (value[0] == '.')
              value = '0' + value;
            ss << value;
            if (i != values.size() - 1)
              ss << ",";
          }
          if (values.size() == 1)
            ss << "\"";
          jsonStr += "\"specular\":[" + ss.str() + "],";
        }

        ConfigNode *depthWriteNode = passNode->FindChild("depth_write");
        if (depthWriteNode)
        {
          std::stringstream ss;
          std::string depthWriteStr = depthWriteNode->Value(0);
          if (depthWriteStr == "off")
            ss << "false";
          else
            ss << "true";
          jsonStr += "\"depth_write\":" + ss.str() + ",";
        }

        ConfigNode *depthCheckNode = passNode->FindChild("depth_check");
        if (depthCheckNode)
        {
          std::stringstream ss;
          std::string depthCheckStr = depthCheckNode->Value(0);
          if (depthCheckStr == "off")
            ss << "false";
          else
            ss << "true";
          jsonStr += "\"depth_check\":" + ss.str() + ",";
        }

        ConfigNode *textureUnitNode = passNode->FindChild("texture_unit");
        if (textureUnitNode)
        {
          ConfigNode *textureNode = textureUnitNode->FindChild("texture");
          if (textureNode)
          {
            std::string textureStr = textureNode->Value(0);
            index = textureStr.rfind(".");
            if (index != std::string::npos)
            {
              textureStr = textureStr.substr(0, index+1) + "png";
            }

            jsonStr += "\"texture\":\"" + textureStr + "\",";
          }
          ConfigNode *scaleNode = textureUnitNode->FindChild("scale");
          if (scaleNode)
          {
            std::stringstream ss;
            std::vector<std::string> values = scaleNode->Values();
            if (values.size() == 1)
              ss << "\"";
            for (unsigned int i = 0; i < values.size(); ++i)
            {
              std::string value = scaleNode->Value(i);
              if (value[0] == '.')
                value = '0' + value;
              ss << value;
              if (i != values.size() - 1)
                ss << ",";
            }
            if (values.size() == 1)
              ss << "\"";
            jsonStr += "\"scale\":[" + ss.str() + "],";
          }
          ConfigNode *alphaOpNode = textureUnitNode->FindChild("alpha_op_ex");
          if (alphaOpNode)
          {
            std::stringstream ss;
            std::vector<std::string> values = alphaOpNode->Values();
            // a bit hacky, just assuming there is an alpha value to use
            // fix this to support more ogre alpha operations.
            if (values[1] == "src_manual")
            {
              ss << values[3];
            }
            jsonStr += "\"opacity\":" + ss.str() + ",";
          }
        }
        if (jsonStr[jsonStr.size()-1] == ',')
        {
          jsonStr = jsonStr.substr(0, jsonStr.size()-1);
        }
        jsonStr += "}";
      }
    }

  }

  jsonStr += "}";

  // std::cout << jsonStr << std::endl;

  return jsonStr;
}


/////////////////////////////////////////////////
ConfigNode::ConfigNode(ConfigNode *_parent, const std::string &_name)
{
  this->name = _name;
  this->parent = _parent;
  //For proper destruction
  this->removeSelf = true;
  this->lastChildFound = -1;

  // add self to parent's child list
  // (unless this is the root node being created)
  if (_parent != NULL)
  {
    this->parent->children.push_back(this);
    this->selfNode = --(this->parent->children.end());
  }
}

/////////////////////////////////////////////////
ConfigNode::~ConfigNode()
{
  // delete all children
  for (auto &c : this->children)
  {
    c->removeSelf = false;
    delete c;
  }
  this->children.clear();

  // remove self from parent's child list
  if (this->removeSelf && this->parent != NULL)
  {
    this->parent->children.erase(this->selfNode);
  }
}

/////////////////////////////////////////////////
ConfigNode *ConfigNode::AddChild(const std::string &_name,
    bool _replaceExisting)
{
  if (_replaceExisting)
  {
    ConfigNode *node = FindChild(_name, false);
    if (node)
    {
      return node;
    }
  }
  return new ConfigNode(this, _name);
}

/////////////////////////////////////////////////
ConfigNode *ConfigNode::FindChild(const std::string &_name, bool _recursive)
{
  int indx, prevC, nextC;
  int childCount = (int)this->children.size();

  if (this->lastChildFound != -1)
  {
    // If possible, try checking the nodes neighboring the last successful search
    // (often nodes searched for in sequence, so this will boost search speeds).
    prevC = this->lastChildFound-1;
    if (prevC < 0)
      prevC = 0;
    else if (prevC >= childCount)
      prevC = childCount-1;

    nextC = this->lastChildFound+1;
    if (nextC < 0)
      nextC = 0;
    else if (nextC >= childCount)
      nextC = childCount-1;

    for (indx = prevC; indx <= nextC; ++indx)
    {
      ConfigNode *node = this->children[indx];
      if (node->name == _name)
      {
        this->lastChildFound = indx;
        return node;
      }
    }

    // if not found that way, search for the node from start to finish,
    // avoiding the already searched area above.
    for (indx = nextC + 1; indx < childCount; ++indx)
    {
      ConfigNode *node = this->children[indx];
      if (node->name == _name) {
        this->lastChildFound = indx;
        return node;
      }
    }
    for (indx = 0; indx < prevC; ++indx)
    {
      ConfigNode *node = this->children[indx];
      if (node->name == _name) {
        this->lastChildFound = indx;
        return node;
      }
    }
  }
  else
  {
    // search for the node from start to finish
    for (indx = 0; indx < childCount; ++indx){
      ConfigNode *node = this->children[indx];
      if (node->name == _name) {
        this->lastChildFound = indx;
        return node;
      }
    }
  }

  // if not found, search child nodes (if recursive == true)
  if (_recursive)
  {
    for (indx = 0; indx < childCount; ++indx)
    {
      this->children[indx]->FindChild(_name, _recursive);
    }
  }

  // not found anywhere
  return NULL;
}

/////////////////////////////////////////////////
void ConfigNode::SetParent(ConfigNode *_newParent)
{
  // remove self from current parent
  this->parent->children.erase(this->selfNode);

  // set new parent
  this->parent = _newParent;

  // add self to new parent
  this->parent->children.push_back(this);
  this->selfNode = --(this->parent->children.end());
}


/////////////////////////////////////////////////
void ConfigNode::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
std::string ConfigNode::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
void ConfigNode::AddValue(const std::string &_value)
{
  this->values.push_back(_value);
}

/////////////////////////////////////////////////
void ConfigNode::ClearValues()
{
  this->values.clear();
}

/////////////////////////////////////////////////
std::vector<std::string> &ConfigNode::Values()
{
  return this->values;
}

/////////////////////////////////////////////////
const std::string &ConfigNode::Value(unsigned int _index)
{
  assert(_index < this->values.size());
  return this->values[_index];
}

/////////////////////////////////////////////////
void ConfigLoader::LoadAllFiles(ConfigLoader* c, const std::string &_path)
{
  try
  {
    for ( boost::filesystem::recursive_directory_iterator end, dir(_path);
        dir != end; ++dir )
    {
      boost::filesystem::path p(*dir);
      if(p.extension() == c->fileEnding)
      {
        std::ifstream in((*dir).path().string().c_str(), std::ios::binary);
        c->ParseScript(in);
      }
    }
  }
  catch (boost::filesystem::filesystem_error &e)
  {
    std::cerr << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////
ConfigLoader::ConfigLoader(const std::string &_fileEnding)
{
  // register as a ScriptLoader
  this->fileEnding = _fileEnding;
}

/////////////////////////////////////////////////
ConfigLoader::~ConfigLoader()
{
  ClearScriptList();
}

/////////////////////////////////////////////////
void ConfigLoader::ClearScriptList()
{
  for (auto &s : this->scriptList)
  {
    delete s.second;
  }
  this->scriptList.clear();
}

/////////////////////////////////////////////////
ConfigNode *ConfigLoader::ConfigScript(const std::string &_name)
{
  auto it = this->scriptList.find(_name);

  // if found..
  if (it != this->scriptList.end())
  {
    return it->second;
  }
  else
  {
    return NULL;
  }
}

/////////////////////////////////////////////////
std::map <std::string, ConfigNode*> ConfigLoader::AllConfigScripts()
{
  return this->scriptList;
}

/////////////////////////////////////////////////
void ConfigLoader::ParseScript(std::ifstream &_stream)
{
  // get first token
  this->NextToken(_stream);
  if (tok == TOKEN_EOF)
  {
    _stream.close();
    return;
  }

  // parse the script
  this->ParseNodes(_stream, 0);

  _stream.close();
}

/////////////////////////////////////////////////
void ConfigLoader::NextToken(std::ifstream &_stream)
{
  lastTok = tok;
  lastTokVal = tokVal;

  // EOF token
  if (_stream.eof())
  {
    tok = TOKEN_EOF;
    return;
  }

  // (get next character)
  int ch = _stream.get();
  if (ch == -1)
  {
    tok = TOKEN_EOF;
    return;
  }
  while ((ch == ' ' || ch == 9) && !_stream.eof())
  {    //Skip leading spaces / tabs
    ch = _stream.get();
  }

  if (_stream.eof())
  {
    tok = TOKEN_EOF;
    return;
  }

  // newline token
  if (ch == '\r' || ch == '\n')
  {
    do
    {
      ch = _stream.get();
    } while ((ch == '\r' || ch == '\n') && !_stream.eof());

    _stream.unget();

    tok = TOKEN_NEWLINE;
    return;
  }

  // open brace token
  else if (ch == '{')
  {
    tok = TOKEN_OPENBRACE;
    return;
  }

  // close brace token
  else if (ch == '}')
  {
    tok = TOKEN_CLOSEBRACE;
    return;
  }

  // text token
  // verify valid char
  if (ch < 32 || ch > 122)
  {
    throw std::runtime_error(
        "Parse Error: Invalid character, ConfigLoader::load()");
  }

  tokVal = "";
  tok = TOKEN_TEXT;
  do
  {
    // skip comments
    if (ch == '/')
    {
      int ch2 = _stream.peek();

      // C++ style comment (//)
      if (ch2 == '/')
      {
        _stream.get();
        do
        {
          ch = _stream.get();
        }
        while (ch != '\r' && ch != '\n' && !_stream.eof());

        tok = TOKEN_NEWLINE;
        return;
      }
      else if (ch2 == '*')
      {
        _stream.get();
        do
        {
          ch = _stream.get();
          ch2 = _stream.peek();
        }
        while (!(ch == '*' && ch2 == '/') && !_stream.eof());
        _stream.get();
        do
        {
          ch = _stream.get();
        }
        while (ch != '\r' && ch != '\n' && !_stream.eof());
        continue;
      }
    }

    // add valid char to tokVal
    tokVal += (char)ch;

    // next char
    ch = _stream.get();

  }
  while (ch > 32 && ch <= 122 && !_stream.eof());

  _stream.unget();

  return;
}

/////////////////////////////////////////////////
void ConfigLoader::SkipNewLines(std::ifstream &_stream)
{
  while (tok == TOKEN_NEWLINE)
  {
    this->NextToken(_stream);
  }
}

/////////////////////////////////////////////////
void ConfigLoader::ParseNodes(std::ifstream &_stream, ConfigNode *_parent)
{
  typedef std::pair<std::string, ConfigNode*> ScriptItem;

  while (true)
  {
    switch (tok)
    {
      // node
      case TOKEN_TEXT:
        // add the new node
        ConfigNode *newNode;
        if (_parent)
        {
          newNode = _parent->AddChild(tokVal);
        }
        else
        {
          newNode = new ConfigNode(0, tokVal);
        }

        // get values
        this->NextToken(_stream);
        while (tok == TOKEN_TEXT)
        {
          newNode->AddValue(tokVal);
          this->NextToken(_stream);
        }

        // add root nodes to scriptList
        if (!_parent)
        {
          std::string key;

          if (newNode->Values().empty())
          {
            key = newNode->Name() + ' ';
          }
          else
          {
            key = newNode->Name() + ' ' + newNode->Values().front();
          }

          this->scriptList.insert(ScriptItem(key, newNode));
        }

        this->SkipNewLines(_stream);

        // add any sub-nodes
        if (tok == TOKEN_OPENBRACE)
        {
          // parse nodes
          this->NextToken(_stream);
          this->ParseNodes(_stream, newNode);
          // check for matching closing brace
          if (tok != TOKEN_CLOSEBRACE)
          {
            throw std::runtime_error("Parse Error: Expecting closing brace");
          }
          this->NextToken(_stream);
          this->SkipNewLines(_stream);
        }

        break;

      // out of place brace
      case TOKEN_OPENBRACE:
        throw std::runtime_error("Parse Error: Opening brace out of plane");
        break;

      // return if end of nodes have been reached
      case TOKEN_CLOSEBRACE:
        return;

      // return if reached end of file
      case TOKEN_EOF:
        return;

      case TOKEN_NEWLINE:
        this->NextToken(_stream);
        break;

      default:
        break;
    }
  };
}
